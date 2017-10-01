#include <Arduino.h>
#include "Feather.h"
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#include "Adafruit_BLEBattery.h"
#include <Button.h>

#define MINIMUM_FIRMWARE_VERSION "0.6.6"
#define MODE_LED_BEHAVIOUR      "MODE"

const unsigned int batt_mv_curve[] = {4200, 4030, 3860, 3830, 3790, 3700, 3600};
const unsigned char batt_charge_curve[] = {100, 76, 52, 42, 30, 11, 0};

const int ledPin = LED_BUILTIN;

bool isConnected = false;

unsigned int cycle_ble = 0;
unsigned int cycle_battery = 0;

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEMIDI midi(ble);
Adafruit_BLEBattery battery(ble);

Button button1(5);
Button button2(6);

#define MIDI_COMMAND_ON     0x90
#define MIDI_COMMAND_OFF    0x80

#define CYCLE_TIME          500 /* ms */
#define CYCLE_TIME_BATTERY  1000

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

}

void setup()
{
  Serial.begin(115200);

  button1.begin();
  button2.begin();

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


void sendCC(unsigned char mute) {
  // Send CC message on channel 0, controller 1
  midi.send(0xB0, 1, mute ? 127 : 0);
}

void loop() {
  if (cycle_ble >= CYCLE_TIME) {
    cycle_ble = 0;
    ble.update(CYCLE_TIME);
  } else {
    cycle_ble += 100;
  }

  if (isConnected) {
    if (cycle_battery >= CYCLE_TIME_BATTERY) {
      cycle_battery = 0;

      unsigned int voltage = getBatteryVoltage();
      unsigned char charge = chargeLookup(batt_mv_curve, batt_charge_curve, voltage);

      Serial.print("VBat: ");
      Serial.print(voltage);
      Serial.print(" c: ");
      Serial.println(charge);

      ble.print("VBat: ");
      ble.print(voltage);
      ble.print(" c: ");
      ble.println(charge);

      battery.update(charge);
    } else {
      cycle_battery += 100;
    }
  }

  // Button 1 toggles the mute button in Reaper
  if (button1.toggled()) {
    if (button1.read() == Button::PRESSED) {
      Serial.println("Button 1 has been pressed");
      if (isConnected) {
        sendCC(1);
      }
    } else {
      Serial.println("Button 1 has been released");
      if (isConnected) {
        sendCC(0);
      }
    }
  }

  // Button 2 sets the marker in Reaper
  if (button2.toggled()) {
    if (button2.read() == Button::PRESSED) {
      Serial.println("Button 2 has been pressed");
      if (isConnected) {
        // Send the note to reaper to set the marker
        midi.send(MIDI_COMMAND_ON, 0, 64);
      }
    } else {
      Serial.println("Button 2 has been released");
      if (isConnected) {
        midi.send(MIDI_COMMAND_OFF, 0, 64);
      }
    }
  }

  delay(100);
}
