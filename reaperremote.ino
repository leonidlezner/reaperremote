/**
   Author: Leonid Lezner
   License: MIT

   Required libraries: Adafruit_BluefruitLE_nRF51, Button
*/

#include <Arduino.h>
#include "Feather.h"
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#include "Adafruit_BLEBattery.h"
#include <Button.h>

#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

const unsigned int batt_mv_curve[] = {4200, 4030, 3860, 3830, 3790, 3700, 3600};
const unsigned char batt_charge_curve[] = {100, 76, 52, 42, 30, 11, 0};

const int ledPin = LED_BUILTIN;

const int ledRed = A3;
const int ledYellow = A4;

#define LED_RED     1
#define LED_YELLOW  2  
#define LED_OFF     0

bool isConnected = false;

unsigned int cycle_ble = 0;
unsigned int cycle_battery = 0;

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEMIDI midi(ble);
Adafruit_BLEBattery battery(ble);

// Button 1 is mute
Button button1(6);

// Button 2 is chapter marker
Button button2(5);

#define CYCLE_TIME_BLE      200 /* ms */
#define CYCLE_TIME_BATTERY  1000 /* ms */
#define CYCLE_TIME          50

void error(const __FlashStringHelper *err)
{
  Serial.println(err);

  while (1)
  {
    // block execution here...
  };
}

void connected(void)
{
  isConnected = true;
}

void disconnected(void)
{
  isConnected = false;
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
  /* no action, ignoring the incomming data */
}

void showLED(unsigned char state)
{
  switch(state)
  {
    case LED_YELLOW:
      digitalWrite(ledYellow, HIGH);
      digitalWrite(ledRed, LOW);
      break;
    case LED_RED:
      digitalWrite(ledYellow, LOW);
      digitalWrite(ledRed, HIGH);
      break;      
    case LED_OFF:
    default:
      digitalWrite(ledYellow, LOW);
      digitalWrite(ledRed, LOW);
  }
}

void setup()
{
  Serial.begin(115200);

  button1.begin();
  button2.begin();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);

  Serial.println(F("Reaper Remote by Allesnetz.com"));

  if (!ble.begin(VERBOSE_MODE))
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  Serial.println(F("OK!"));

  ble.echo(false);

  if (!ble.sendCommandCheckOK(F("AT+GAPDEVNAME=ReaperRemote")))
  {
    error(F("Could not set device name"));
  }

  Serial.println("Requesting Bluefruit info:");

  ble.info();

  ble.setConnectCallback(connected);

  ble.setDisconnectCallback(disconnected);

  midi.setRxCallback(BleMidiRX);

  if (!midi.begin(true))
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);

  // LED Activity command is only supported from 0.6.6
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
  {
    ble.sendCommandCheckOK("AT+HWMODELED=" MODE_LED_BEHAVIOUR);
  }

  Serial.print(F("Waiting for a connection..."));

  ble.setMode(BLUEFRUIT_MODE_DATA);

  // Start the Battery GATT service
  battery.begin(true);

  if (!ble.sendCommandCheckOK(F("AT+GAPSTARTADV")))
  {
    error(F("Could not start advertising"));
  }
}

void sendMuteCommand(bool mute) {
  // Send CC message on channel 0, controller 1
  midi.send(0xB0, 1, mute ? 127 : 0);
}

void sendMidiNote(bool onState, unsigned char channel, unsigned char note, unsigned char velocity) {
  midi.send((onState ? 0x90 : 0x80) | (channel & 0x0F), note, velocity);
}

void loop() {
  if (cycle_ble >= CYCLE_TIME_BLE) {
    cycle_ble = 0;
    ble.update(CYCLE_TIME_BLE);
  } else {
    cycle_ble += CYCLE_TIME;
  }

  if (isConnected) {
    if (cycle_battery >= CYCLE_TIME_BATTERY) {
      cycle_battery = 0;

      unsigned int voltage = getBatteryVoltage();
      unsigned char charge = chargeLookup(batt_mv_curve, batt_charge_curve, voltage);

      ble.print("VBat: ");
      ble.print(voltage);
      ble.print(" c: ");
      ble.println(charge);

      battery.update(charge);
    } else {
      cycle_battery += CYCLE_TIME;
    }
  }

  // Button 1 toggles the mute button in Reaper
  if (button1.toggled()) {
    if (button1.read() == Button::PRESSED) {
      Serial.println("Button 1 has been pressed");
      showLED(LED_RED);
      if (isConnected) {
        sendMuteCommand(true);
      }
    } else {
      Serial.println("Button 1 has been released");
      showLED(LED_OFF);
      if (isConnected) {
        sendMuteCommand(false);
      }
    }
  }

  // Button 2 sets the marker in Reaper
  if (button2.toggled()) {
    if (button2.read() == Button::PRESSED) {
      Serial.println("Button 2 has been pressed");
      showLED(LED_YELLOW);
      if (isConnected) {
        // Send the note to reaper to set the marker
        sendMidiNote(true, 0, 0, 64);
      }
    } else {
      Serial.println("Button 2 has been released");
      showLED(LED_OFF);
      if (isConnected) {
        sendMidiNote(false, 0, 0, 64);
      }
    }
  }

  delay(CYCLE_TIME);
}
