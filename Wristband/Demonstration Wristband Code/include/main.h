#ifndef _main_H
#define _main_H

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <chrono>
#include <HardwareSerial.h>
#include <stdlib.h>
#include <string>
#include <Wire.h>

#define CNTL1 0x18         // Control register 1
#define FFCNTL 0x2E        // Free-Fall Control Register
#define FFTH 0x2C          // Free-Fall Threshold Register
#define FFC 0x2D           // Free-Fall Counter Register
#define INC1 0x1C          // Interrupt Control Register 1
#define INC4 0x1F          // Interrupt Control Register 4
#define INT_REL 0x17       // Interrupt Release Register
#define INT1_PIN 32        // INT1 pin connected to pin 32 of the microcontroller
#define KX122_ADDRESS 0x1F // Address of the KX122 accelerometer
#define MANUAL_HR_PIN 37   // Button to set HR to 20
#define RESET_PIN 38       // Pin to reset all flags
#define SDA 21
#define SCL 22
#define BTDISCONNECTPIN 27

#define RED_PIN 15
#define GREEN_PIN 2
#define BLUE_PIN 4

void CheckHR();

void InitializeAccel();

void InitializeSensor();

void Int1_ISR();

void PrintBTMAC(bool);

void ReadByte(uint8_t);

void ReconnectBluetooth(bool);

void SendOK();

void SendWarn();

void SetColor(int, int, int);

void WriteByte(uint8_t, uint8_t);

#endif