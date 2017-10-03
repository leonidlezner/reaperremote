/**
   Author: Leonid Lezner
   License: MIT

   Required libraries: Adafruit_BluefruitLE_nRF51, Button
*/

#include <Arduino.h>
#include "Feather.h"
#include "remote.h"
#include "at_parser.h"
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#include "Adafruit_BLEBattery.h"
#include <Button.h>



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

MIDIDATA button1_data;
MIDIDATA button2_data;

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

char getButtonDown(char *value)
{
  Serial.println("getButtonDown");
  return AT_OK;
}

char setButtonDown(char *value)
{
  Serial.println("setButtonDown");
  return AT_OK;
}

char getButtonUp(char *value)
{
  Serial.println("getButtonUp");
  return AT_OK;
}

char setButtonUp(char *value)
{
  Serial.println("setButtonUp");
  return AT_OK;
}

void setup()
{
  Serial.begin(115200);

  button1.begin();
  button2.begin();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

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

  at_register_command((string_t)"BTND", (at_callback)getButtonDown, (at_callback)setButtonDown, 0, 0);
  at_register_command((string_t)"BTNU", (at_callback)getButtonUp, (at_callback)setButtonUp, 0, 0);
}

void sendMidiData(MIDIDATA *data) {
  midi.send(data->bytes[0], data->bytes[1], data->bytes[2]);
}

void sendMuteCommand(bool mute) {
  // Send CC message on channel 0, controller 1
  // midi.send(, 1, mute ? 127 : 0);

  button1_data.bytes[0] = 0xB0;
  button1_data.bytes[1] = 0x01;
  button1_data.bytes[2] = mute ? 0x7F : 0;
  sendMidiData(&button1_data);
}

void sendMidiNote(bool onState, unsigned char channel, unsigned char note, unsigned char velocity) {
  //midi.send((onState ? 0x90 : 0x80) | (channel & 0x0F), note, velocity);

  button1_data.bytes[0] = (onState ? 0x90 : 0x80) | (channel & 0x0F);
  button1_data.bytes[1] = note;
  button1_data.bytes[2] = velocity;
  sendMidiData(&button1_data);
}

void process_at_commands()
{
  static String readString = "";
  char ret[50];
  char res;

  while (Serial.available())
  {
    if (Serial.available() > 0)
    {
      // Get a byte from buffer
      char c = Serial.read();

      readString += c;

      // Input is too long
      if (readString.length() > AT_MAX_TEMP_STRING)
      {
        Serial.println(AT_ERROR_STRING);
        readString = "";
      }
      else
      {
        if (c == '\r' || c == ';')
        {
          readString.trim();

          // Simple echo
          if (readString == "AT")
          {
            Serial.println(DEVICE_DESCRIPTION);
            Serial.println(AT_OK_STRING);
            readString = "";
          }
          else
          {
            // Parsing the command
            res = at_parse_line((string_t)readString.c_str(), (unsigned char*)ret);

            readString = "";

            if (res == AT_OK)
            {
              if (ms_strlen((string_t)ret) > 0)
              {
                String s_ret(ret);
                Serial.println(s_ret);
              }
              Serial.println(AT_OK_STRING);
            }
            else
            {
              Serial.println(AT_ERROR_STRING);
            }
          }
        }
      }
    } // end serial available
  } // end while
}

void loop() {
  process_at_commands();

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
      digitalWrite(ledPin, HIGH);
      if (isConnected) {
        sendMuteCommand(true);
      }
    } else {
      Serial.println("Button 1 has been released");
      digitalWrite(ledPin, LOW);
      if (isConnected) {
        sendMuteCommand(false);
      }
    }
  }

  // Button 2 sets the marker in Reaper
  if (button2.toggled()) {
    if (button2.read() == Button::PRESSED) {
      Serial.println("Button 2 has been pressed");
      digitalWrite(ledPin, HIGH);
      if (isConnected) {
        // Send the note to reaper to set the marker
        sendMidiNote(true, 0, 0, 64);
      }
    } else {
      Serial.println("Button 2 has been released");
      digitalWrite(ledPin, LOW);
      if (isConnected) {
        sendMidiNote(false, 0, 0, 64);
      }
    }
  }

  delay(CYCLE_TIME);
}
