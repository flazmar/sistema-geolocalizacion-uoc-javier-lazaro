# TFG: Sistema de apertura inteligente basado en localización y biometría de un armero militar

This project was developed by Francisco Javier Lázaro Martínez at [Universitat Oberta de Catalunya (UOC)](https://www.uoc.edu). 

The implementation, analysis, and results are the author's original work.

---

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [File Structure](#file-structure)
4. [License](#license)


---

## Introduction

This repository contains the development of a functional prototype for a dual-factor authentication system aimed at improving access control to sensitive material in military environments.
The system combines GPS-based geolocation and biometric fingerprint authentication to ensure that access is only granted when both the user identity and the authorized geographic position are validated.

The prototype is implemented on an ESP32 microcontroller and follows a modular hardware and software architecture, integrating multiple peripherals such as a GPS module, biometric sensor, real-time clock (RTC), LCD display, microSD storage and a local web server.

## Features

Dual-factor authentication system:

  GPS geolocation validation
  Biometric fingerprint authentication

Modular hardware architecture based on ESP32.
Modular software design with independent functional modules.
Local WiFi access point with integrated web server.
Web interface for:

  Authorized GPS area configuration
  User (fingerprint) management
  System status monitoring

Event logging system using microSD storage for audit purposes.
Real-time clock synchronization via GPS and RTC module.
LCD-based local user interface.
Designed for traceability, auditability and future extensibility.


## File Structure
- sketch: final version Arduino IDE.

## License
This project is licensed under the MIT License - see the LICENSE file for details.
