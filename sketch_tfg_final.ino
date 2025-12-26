// ---------- BLOQUE 1: LIBRERIAS ----------

#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <ESP32Servo.h>
#include <Adafruit_Fingerprint.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

// ---------- BLOQUE 2: DEFINICIÓN PARÁMETROS ----------


// ---------- CONFIGURACIÓN WIFI ----------
const char* AP_SSID     = "RAAA71";
const char* AP_PASSWORD = "12345678";

// ---------- CONFIGURACIÓN GPS ----------
static const int GPS_RX_PIN = 16;  // NOT NECESSARY
static const int GPS_TX_PIN = 17;  // GPS TX
static const long GPS_BAUD  = 9600;

// ---------- SERVO CERRADURA ----------
const int SERVO_PIN        = 14;
const int SERVO_LOCK_POS   = 0;    // GRADES CLOSE
const int SERVO_UNLOCK_POS = 90;   // GRADES OPEN

Servo lockServo;
bool servoOpen = false;            // FALSE = CLOSE

// ---------- LCD I2C ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool lcdReady = false;

// ---------- RTC DS3231 ----------
RTC_DS3231 rtc;
bool rtcOk = false;

// ---------- SD ----------
const int SD_CS_PIN = 5;
bool sdOk = false;

// ---------- GPS  ----------
HardwareSerial SerialGPS(2);
TinyGPSPlus gps;

// ---------- WEB SERVER ----------
WebServer server(80);  // PORT

// ---------- POSICIONES AUTORIZADAS ----------
const int MAX_AUTH_POS = 10;   // MAX POSITIONS AUTHORIZED
int     authorizedCount = 0;
double  authorizedLat[MAX_AUTH_POS];
double  authorizedLon[MAX_AUTH_POS];

// Radio de autorización (en metros)
const double AUTH_RADIUS_M = 2000.0;   // 2KM RADIUS AUTHORIZED

// ---------- CONFIGURACIÓN LECTOR HUELLA ----------
static const int FINGER_RX_PIN = 25;    //  TX
static const int FINGER_TX_PIN = 26;    //  RX
static const uint32_t FINGER_BAUD = 57600;

HardwareSerial FingerSerial(1);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&FingerSerial);
bool fingerOk = false;

// ---------- ESTADO UI AUTENTICACIÓN ----------
enum AuthUIState {
  AUTH_WAIT = 0,
  AUTH_ERROR,
  AUTH_OPEN
};

AuthUIState authState = AUTH_WAIT;
unsigned long lastErrorMs = 0;

// ---------- NUEVA HUELLA DESDE WEB ----------
bool enrollRequested = false;

// ---------- PROTECCIÓN ANTI-SPAM HUELLA ----------
const unsigned long FINGER_COOLDOWN_MS = 10000;  // 10s AFTER FAIL
unsigned long lastFingerFailMs = 0;

// ---------- BLOQUE 3: FUNCIONES AUXILIARES ----------

// ---------- COMPROBAR FIX ----------
bool hasFix() {
  if (!gps.location.isValid()) return false;
  if (!gps.satellites.isValid()) return false;
  if (gps.satellites.value() < 4) return false;
  if (gps.location.age() > 2000) return false;
  return true;
}

// ---------- DATOS TIEMPO ----------

String formatDateTime(const DateTime& dt) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(),
           dt.hour(), dt.minute(), dt.second());
  return String(buf);
}

// ---------- DISTANCIA ENTRE 2 COORDENADAS (HAVERSINE) ----------

double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // radio Tierra en m
  double radLat1 = lat1 * PI / 180.0;
  double radLat2 = lat2 * PI / 180.0;
  double dLat    = (lat2 - lat1) * PI / 180.0;
  double dLon    = (lon2 - lon1) * PI / 180.0;

  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(radLat1) * cos(radLat2) *
             sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

// ---------- POS ACTUAL AUTORIZADA ----------

bool isInAuthorizedArea() {
  if (!hasFix()) return false;
  if (authorizedCount == 0) return false;
  if (!gps.location.isValid()) return false;

  double lat = gps.location.lat();
  double lon = gps.location.lng();

  for (int i = 0; i < authorizedCount; i++) {
    double d = distanceMeters(lat, lon, authorizedLat[i], authorizedLon[i]);
    if (d <= AUTH_RADIUS_M) {
      return true;
    }
  }
  return false;
}

// ---------- SD: inicialización y reintento ----------

bool initSD() {
  Serial.println("[SD] Iniciando SD...");
  SPI.begin(18, 19, 23, SD_CS_PIN);
  if (SD.begin(SD_CS_PIN, SPI, 1000000)) {
    sdOk = true;
    Serial.println("[SD] SD iniciada correctamente.");
    if (!SD.exists("/events.txt")) {
      File f = SD.open("/events.txt", FILE_WRITE);
      if (f) {
        f.println("datetime;type;detail");
        f.close();
      }
    }
  } else {
    sdOk = false;
    Serial.println("[SD] ERROR: No se pudo iniciar la SD.");
  }
  return sdOk;
}

bool ensureSD() {
  if (sdOk) return true;

  static unsigned long lastTry = 0;
  unsigned long now = millis();
  if (now - lastTry < 5000) return false;
  lastTry = now;

  return initSD();
}

// ---------- LOG A SD ----------

void logEvent(const String& type, const String& detail) {
  if (!rtcOk) {
    Serial.print("[LOG] ");
    Serial.print(type);
    Serial.print(" ; ");
    Serial.println(detail);
    return;
  }

  DateTime now = rtc.now();
  String line = formatDateTime(now) + ";" + type + ";" + detail;

  if (!ensureSD()) {
    Serial.print("[SD] No disponible, NO se loguea -> ");
    Serial.println(line);
    return;
  }

  File f = SD.open("/events.txt", FILE_APPEND);
  if (!f) {
    Serial.println("[SD] Error abriendo /events.txt para escribir sdOk=false.");
    sdOk = false;
    return;
  }

  f.println(line);
  f.close();

  Serial.print("[LOG] ");
  Serial.println(line);
}

// ---------- LCD FECHA (CON GPS OFF) ----------

void updateLcdDateLine() {
  if (!lcdReady || !rtcOk) return;

  DateTime now = rtc.now();
  char buf[17];
  snprintf(buf, sizeof(buf), "%02d/%02d/%04d",
           now.day(), now.month(), now.year());
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

// ---------- SERVO ----------

void setLock(bool open) {
  servoOpen = open;
  if (!lockServo.attached()) return;

  if (open) {
    lockServo.write(SERVO_UNLOCK_POS);
    logEvent("SERVO", "OPEN");
  } else {
    lockServo.write(SERVO_LOCK_POS);
    logEvent("SERVO", "CLOSE");
  }
}

// ---------- RTC desde PC ----------

void updateRTCfromPC() {
  if (!rtcOk) return;
  DateTime pcTime(F(__DATE__), F(__TIME__));
  rtc.adjust(pcTime);

  Serial.println("[RTC] Actualizado desde PC:");
  Serial.print("       ");
  Serial.print(pcTime.day());   Serial.print("/");
  Serial.print(pcTime.month()); Serial.print("/");
  Serial.print(pcTime.year());  Serial.print("  ");
  Serial.print(pcTime.hour());  Serial.print(":");
  Serial.print(pcTime.minute());Serial.print(":");
  Serial.println(pcTime.second());

  logEvent("RTC_FROM_PC", formatDateTime(pcTime));
}

// ---------- RTC desde GPS (UTC+1) ----------

void updateRTCfromGPS() {
  if (!rtcOk) return;
  if (!gps.date.isValid() || !gps.time.isValid()) return;

  int year   = gps.date.year();
  int month  = gps.date.month();
  int day    = gps.date.day();

  int hour   = gps.time.hour() + 1; // SPAIN: UTC+1
  if (hour >= 24) hour -= 24;
  int minute = gps.time.minute();
  int second = gps.time.second();

  DateTime gpsTime(year, month, day, hour, minute, second);
  rtc.adjust(gpsTime);

  Serial.println("[RTC] Actualizado desde GPS (UTC+1):");
  Serial.print("       ");
  Serial.print(day);   Serial.print("/");
  Serial.print(month); Serial.print("/");
  Serial.print(year);  Serial.print("  ");
  Serial.print(hour);  Serial.print(":");
  Serial.print(minute);Serial.print(":");
  Serial.println(second);

  logEvent("RTC_FROM_GPS", formatDateTime(gpsTime));
}

// ---------- LCD: UI según estado ----------

void updateLcdUI(bool fix, bool inArea) {
  if (!lcdReady) return;

  static unsigned long lastUpdate = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastUpdate < 300) return;
  lastUpdate = nowMs;

  if (!fix) {
    lcd.setCursor(0, 0);
    lcd.print("RAAA 71 GPS OFF");
    updateLcdDateLine();
    return;
  }

// ---------- LINEA SUPERIOR ----------
  lcd.setCursor(0, 0);
  char top[17] = {0};
  if (rtcOk) {
    DateTime now = rtc.now();
    snprintf(top, sizeof(top), "RAAA 71 %02d:%02d", now.hour(), now.minute());
  } else {
    snprintf(top, sizeof(top), "RAAA 71 --:--");
  }
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(top);

// ---------- LINEA INFERIOR ----------
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);

  if (!inArea) {
    lcd.print("FUERA AREA GPS ");
    authState = AUTH_WAIT;
    return;
  }

// ---------- RESET ----------
  if (authState == AUTH_ERROR && (millis() - lastErrorMs > 3000)) {
    authState = AUTH_WAIT;
  }

  switch (authState) {
    case AUTH_WAIT:
      lcd.print("INSERTE HUELLA");
      break;
    case AUTH_ERROR:
      lcd.print("ERROR AUTENT.");
      break;
    case AUTH_OPEN:
      lcd.print("CANDADO ABIERTO");
      break;
  }
}

// ---------- HUELLA ESPERAR DEDO ----------

void waitFingerRemoved() {
  if (!fingerOk) return;
  while (true) {
    uint8_t p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) break;
    delay(50);
  }
}

// ---------- CONTADOR HUELLAS ----------

int getFingerprintCount() {
  if (!fingerOk) return -1;
  if (finger.getTemplateCount() != FINGERPRINT_OK) return -1;
  return finger.templateCount;
}

// ---------- LECTURA DE HUELLA + LÓGICA DE SERVO ----------
void handleFingerprint(bool fix, bool inArea) {

  if (!fix) return;        // Necesita GPS OK
  if (!fingerOk) return;   // Necesita sensor OK

  unsigned long nowMs = millis();

// ---------- ANTI-SPAM ----------
  if (nowMs - lastFingerFailMs < FINGER_COOLDOWN_MS) {
    return;
  }

// ---------- NO ABRE SINO AUTH AREA ----------
  if (!inArea) {
    uint8_t p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) {
      return;
    }

    authState = AUTH_ERROR;
    lastErrorMs = millis();
    logEvent("FINGER_BLOCK", "OUT_OF_AREA");
    lastFingerFailMs = millis();
    waitFingerRemoved();
    return;
  }

// ---------- LOGICA OK CON AUTH AREA ----------
  uint8_t p = finger.getImage();

// ---------- NO DEDO = NO INTENTO ----------
  if (p == FINGERPRINT_NOFINGER) {
    return;
  }

// ---------- MAL IMAGEN DE DEDO = FALLO ----------
  if (p != FINGERPRINT_OK) {
    authState = AUTH_ERROR;
    lastErrorMs = millis();
    logEvent("FINGER_FAIL", "GET_IMAGE");

    lastFingerFailMs = millis();
    waitFingerRemoved();
    return;
  }

// ---------- PROCESAMIENTO IMAGEN DEDO ----------
  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) {
    authState = AUTH_ERROR;
    lastErrorMs = millis();
    logEvent("FINGER_FAIL", "IMAGE2TZ");

    lastFingerFailMs = millis();
    waitFingerRemoved();
    return;
  }

// ---------- SEARCH ----------
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {

    // ===== HUELLA CORRECTA =====
    String detail = "ID=" + String(finger.fingerID)
                    + ",score=" + String(finger.confidence);

    if (!servoOpen) {
      setLock(true);
      authState = AUTH_OPEN;
      logEvent("FINGER_OK", "OPEN;" + detail);
    } else {
      setLock(false);
      authState = AUTH_WAIT;
      logEvent("FINGER_OK", "CLOSE;" + detail);
    }

    waitFingerRemoved();
    return;
  }

  // ===== HUELLA INCORRECTA =====
  authState = AUTH_ERROR;
  lastErrorMs = millis();
  logEvent("FINGER_FAIL", "NOT_FOUND");

  lastFingerFailMs = millis();
  waitFingerRemoved();
}

// ---------- ALTA HUELLA ----------

void enrollNewFingerprint() {
  if (!fingerOk) return;

  if (finger.getTemplateCount() != FINGERPRINT_OK) {
    Serial.println("[FINGER] No se pudo leer numero de plantillas.");
    logEvent("FINGER_ENROLL_FAIL", "GET_COUNT");
    return;
  }

  int id = finger.templateCount + 1;
  if (id <= 0) id = 1;

  Serial.print("[FINGER] Iniciando alta de nueva huella en ID ");
  Serial.println(id);
  logEvent("FINGER_ENROLL_REQ", "ID=" + String(id));

  // ---------- PRIMERA IMAGEN ----------
  lcd.setCursor(0, 1);
  lcd.print("PONGA EL DEDO   ");
  Serial.println("[FINGER] Esperando dedo (imagen 1)...");

  uint8_t p = 0;

  while (true) {
    p = finger.getImage();

    if (p == FINGERPRINT_OK) {
      Serial.println("[FINGER] Imagen 1 OK");
      break;
    }

    if (p == FINGERPRINT_NOFINGER) {
      delay(50);
      continue;
    }

    Serial.print("[FINGER] getImage(1) = ");
    Serial.println(p);
    delay(50);
  }

  p = finger.image2Tz(1);
  if (p != FINGERPRINT_OK) {
    Serial.print("[FINGER] Error procesando imagen 1, code=");
    Serial.println(p);
    logEvent("FINGER_ENROLL_FAIL", "IMAGE2TZ1");
    return;
  }

  // ---------- RETIRAR DEDO ----------
  lcd.setCursor(0, 1);
  lcd.print("RETIRE EL DEDO  ");
  Serial.println("[FINGER] Esperando retirada...");

  while (true) {
    p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) {
      break;
    }
    delay(50);
  }

  // ---------- SEGUNDA IMAGEN ----------
  lcd.setCursor(0, 1);
  lcd.print("MISMO DEDO...   ");
  Serial.println("[FINGER] Esperando dedo (imagen 2)...");

  while (true) {
    p = finger.getImage();

    if (p == FINGERPRINT_OK) {
      Serial.println("[FINGER] Imagen 2 OK");
      break;
    }

    if (p == FINGERPRINT_NOFINGER) {
      delay(50);
      continue;
    }

    Serial.print("[FINGER] getImage(2) = ");
    Serial.println(p);
    delay(50);
  }

  p = finger.image2Tz(2);
  if (p != FINGERPRINT_OK) {
    Serial.print("[FINGER] Error procesando imagen 2, code=");
    Serial.println(p);
    logEvent("FINGER_ENROLL_FAIL", "IMAGE2TZ2");
    return;
  }

  // ---------- CREAR MODELO ----------
  p = finger.createModel();
  if (p != FINGERPRINT_OK) {
    Serial.print("[FINGER] Error creando modelo, code=");
    Serial.println(p);
    logEvent("FINGER_ENROLL_FAIL", "CREATE_MODEL");
    return;
  }

  // ---------- GUARDAR ----------
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.println("[FINGER] Huella almacenada correctamente!");
    lcd.setCursor(0, 1);
    lcd.print("HUELLA GUARDADA ");
    logEvent("FINGER_ENROLL_OK", "ID=" + String(id));
  } else {
    Serial.print("[FINGER] Error guardando plantilla, code=");
    Serial.println(p);
    logEvent("FINGER_ENROLL_FAIL", "STORE_MODEL");
  }

  delay(2000);
  authState = AUTH_WAIT;
}

// ---------- BLOQUE 4: WEB ----------

void handleRoot() {
  bool fix    = hasFix();
  bool inArea = isInAuthorizedArea();

  String html;

  html += "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<meta http-equiv='refresh' content='20'>";
  html += "<title>ARMERIA GAAAL I/71</title>";
  html += "<style>"
          "body{font-family:Arial;background:#111;color:#eee;padding:20px;}"
          ".card{background:#222;padding:15px;border-radius:10px;margin-bottom:10px;}"
          ".ok{color:#4CAF50;font-weight:bold;}"
          ".bad{color:#f44336;font-weight:bold;}"
          ".warn{color:#FFC107;font-weight:bold;}"
          ".danger{background:#e53935;color:#fff;padding:6px;border:none;border-radius:6px;}"
          "button{padding:7px 14px;border:none;border-radius:6px;margin-top:6px;}"
          "input{margin:3px 0;}"
          "</style>";
  html += "</head><body>";

  html += "<h1>ARMERIA GAAAL I/71</h1>";

  // Estado general
  html += "<div class='card'>";
  html += "<h2>Estado general</h2>";
  html += "<p>Hora RTC: ";
  if (rtcOk) {
    DateTime now = rtc.now();
    if (now.hour()   < 10) html += "0";
    html += String(now.hour()) + ":";
    if (now.minute() < 10) html += "0";
    html += String(now.minute()) + ":";
    if (now.second() < 10) html += "0";
    html += String(now.second());
  } else {
    html += "N/A";
  }
  html += "</p>";

  html += "<p>Blackbox (SD): ";
  if (sdOk) {
    html += "<span class='ok'>Operativa</span>";
  } else {
    html += "<span class='bad'>Off</span>";
  }
  html += "</p>";

  html += "<p>Cerradura: ";
  html += servoOpen ? "<span class='ok'>ABIERTA</span>" : "<span class='bad'>CERRADA</span>";
  html += "</p>";

  html += "</div>";

  // GPS + Área autorizada
  html += "<div class='card'>";
  html += "<h2>GPS</h2>";
  html += "<p>GPS: ";
  html += fix ? "<span class='ok'>FIX OK</span>" : "<span class='bad'>SIN FIX</span>";
  html += "</p>";

  html += "<p>Satélites: ";
  html += gps.satellites.isValid() ? String(gps.satellites.value()) : "N/A";
  html += "</p>";

  html += "<p>Latitud: ";
  html += gps.location.isValid() ? String(gps.location.lat(), 6) : "N/A";
  html += "</p>";

  html += "<p>Longitud: ";
  html += gps.location.isValid() ? String(gps.location.lng(), 6) : "N/A";
  html += "</p>";

  html += "<p>Área autorizada: ";
  if (!fix || authorizedCount == 0) {
    html += "<span class='warn'>No evaluable</span>";
  } else if (inArea) {
    html += "<span class='ok'>DENTRO (~2 km)</span>";
  } else {
    html += "<span class='bad'>FUERA</span>";
  }
  html += "</p>";

  html += "</div>";

  // Posiciones autorizadas
  html += "<div class='card'>";
  html += "<h2>Posiciones autorizadas</h2>";
  html += "<p>Total: " + String(authorizedCount) + " / " + String(MAX_AUTH_POS) + "</p>";

  for (int i = 0; i < authorizedCount; i++) {
    html += "<p>" + String(authorizedLat[i], 6) + " , " + String(authorizedLon[i], 6) + "</p>";
  }

  if (authorizedCount > 0) {
    html += "<form action='/clear_all'><button class='danger'>Borrar todas</button></form>";
  }

  html += "<form action='/set_current'><button>Añadir posición actual</button></form>";
  html += "<br>";

  html += "<form action='/set_manual'>";
  html += "Latitud:<br><input name='lat'><br>";
  html += "Longitud:<br><input name='lon'><br>";
  html += "<button>Añadir manual</button>";
  html += "</form>";
  html += "</div>";

  // Huella
  html += "<div class='card'>";
  html += "<h2>Huella dactilar</h2>";

  html += "<p>Sensor: ";
  if (fingerOk) {
    html += "<span class='ok'>Operativo</span>";
  } else {
    html += "<span class='bad'>No disponible</span>";
  }
  html += "</p>";

  int fCount = getFingerprintCount();
  html += "<p>Huellas registradas: ";
  if (!fingerOk || fCount < 0) {
    html += "N/A";
  } else {
    html += String(fCount);
  }
  html += "</p>";

  html += "<form action='/finger_enroll'><button>Registrar nueva huella</button></form>";
  html += "<form action='/finger_clear'><button class='danger'>Borrar todas las huellas</button></form>";

  html += "</div>";

  html += "</body></html>";

  server.send(200, "text/html", html);
}

// Handlers web

void handleSetManual() {
  if (authorizedCount < MAX_AUTH_POS &&
      server.hasArg("lat") && server.hasArg("lon")) {

    float lat = server.arg("lat").toFloat();
    float lon = server.arg("lon").toFloat();

    authorizedLat[authorizedCount] = lat;
    authorizedLon[authorizedCount] = lon;
    authorizedCount++;

    Serial.println("[WEB] Añadida posición manual.");
    String detail = String(lat, 6) + "," + String(lon, 6);
    logEvent("ADD_MANUAL", detail);
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "");
}

void handleSetCurrent() {
  if (authorizedCount < MAX_AUTH_POS && hasFix()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();

    authorizedLat[authorizedCount] = lat;
    authorizedLon[authorizedCount] = lon;
    authorizedCount++;

    Serial.println("[WEB] Añadida posición actual.");
    String detail = String(lat, 6) + "," + String(lon, 6);
    logEvent("ADD_CURRENT", detail);
  } else {
    Serial.println("[WEB] No se puede añadir posición actual.");
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "");
}

void handleClearAll() {
  authorizedCount = 0;
  Serial.println("[WEB] Lista de posiciones borrada.");
  logEvent("CLEAR_ALL", "-");
  server.sendHeader("Location", "/", true);
  server.send(302, "");
}

void handleFingerEnroll() {
  if (fingerOk) {
    enrollRequested = true;
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "");
}

void handleFingerClear() {
  if (fingerOk) {
    uint8_t p = finger.emptyDatabase();
    if (p == FINGERPRINT_OK) {
      Serial.println("[FINGER] Base de datos de huellas borrada.");
      logEvent("FINGER_CLEAR_ALL", "-");
    } else {
      Serial.println("[FINGER] Error borrando base de datos.");
      logEvent("FINGER_ENROLL_FAIL", "CLEAR_DB");
    }
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "");
}

// ---------- BLOQUE 5: FUNCION SETUP ----------

void setup() {
  Serial.begin(115200);
  delay(300);

  // I2C + LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcdReady = true;
  lcd.setCursor(0, 0);
  lcd.print("GPS SYSTEM");
  lcd.setCursor(0, 1);
  lcd.print("BOOTING...");

  // RTC
  if (rtc.begin()) {
    rtcOk = true;
    Serial.println("[RTC] DS3231 detectado.");
    if (rtc.lostPower()) {
      Serial.println("[RTC] Aviso: perdida energetica, ajustando hora.");
    }
    updateRTCfromPC();
  } else {
    rtcOk = false;
    Serial.println("[RTC] ERROR: No detecta DS3231.");
  }

  // SD
  initSD();
  if (sdOk && rtcOk) {
    logEvent("BOOT", "-");
  }

  // GPS
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  lockServo.setPeriodHertz(50);
  lockServo.attach(SERVO_PIN);
  setLock(false); 		// CLOSE AT THE INIT

  // Fingerprint
  FingerSerial.begin(FINGER_BAUD, SERIAL_8N1, FINGER_RX_PIN, FINGER_TX_PIN);
  finger.begin(FINGER_BAUD);
  if (finger.verifyPassword()) {
    fingerOk = true;
    Serial.println("[FINGER] Sensor de huella detectado.");
  } else {
    fingerOk = false;
    Serial.println("[FINGER] ERROR: no se detecta sensor de huella.");
  }

  // WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  // Web
  server.on("/", handleRoot);
  server.on("/set_manual", handleSetManual);
  server.on("/set_current", handleSetCurrent);
  server.on("/clear_all", handleClearAll);
  server.on("/finger_enroll", handleFingerEnroll);
  server.on("/finger_clear", handleFingerClear);
  server.begin();

  Serial.println("[SYS] Sistema iniciado.");
}

// ---------- BLOQUE : FUNCION LOOP ----------

void loop() {
  // GPS
  while (SerialGPS.available()) gps.encode(SerialGPS.read());

  bool fix    = hasFix();
  bool inArea = isInAuthorizedArea();

 // ---------- RTC GPS ----------
  static bool rtcUpdatedOnce = false;
  if (fix && !rtcUpdatedOnce) {
    updateRTCfromGPS();
    rtcUpdatedOnce = true;
  }

 // ---------- HUELLA + SERVO + AREA ----------
  handleFingerprint(fix, inArea);

 // ---------- ALTA HUELLA PÁGINA WEB ----------
  if (enrollRequested && fix && fingerOk) {
    enrollRequested = false;
    enrollNewFingerprint();
  }

 // ---------- ACT LCD ----------
  updateLcdUI(fix, inArea);

  // ---------- WEB ----------
  server.handleClient();

 // ---------- DEBUG 30S ----------
  static unsigned long lastDebug = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastDebug > 30000) {
    lastDebug = nowMs;

    Serial.print("[DBG] RTC: ");
    if (rtcOk) {
      DateTime now = rtc.now();
      if (now.hour()   < 10) Serial.print('0');
      Serial.print(now.hour());   Serial.print(':');
      if (now.minute() < 10) Serial.print('0');
      Serial.print(now.minute()); Serial.print(':');
      if (now.second() < 10) Serial.print('0');
      Serial.print(now.second());
    } else {
      Serial.print("N/A");
    }

    Serial.print(" | Sats: ");
    if (gps.satellites.isValid()) Serial.print(gps.satellites.value());
    else                          Serial.print("N/A");

    Serial.print(" | FIX: ");
    Serial.print(fix ? "SI" : "NO");

    Serial.print(" | Lat: ");
    if (gps.location.isValid()) Serial.print(gps.location.lat(), 6);
    else                        Serial.print("N/A");

    Serial.print(" | Lon: ");
    if (gps.location.isValid()) Serial.print(gps.location.lng(), 6);
    else                        Serial.print("N/A");

    Serial.print(" | Auth count: ");
    Serial.print(authorizedCount);

    Serial.print(" | InArea: ");
    Serial.print(inArea ? "SI" : "NO");

    Serial.print(" | SD: ");
    Serial.print(sdOk ? "ok" : "off");

    Serial.print(" | Servo: ");
    Serial.print(servoOpen ? "OPEN" : "CLOSED");

    Serial.print(" | Finger: ");
    Serial.print(fingerOk ? "OK" : "FAIL");

    Serial.println();
  }
}
