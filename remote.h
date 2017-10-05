#ifndef __REMOTE_H_
#define __REMOTE_H_

#include "at_parser.h"

typedef struct _mididata {
  unsigned char bytes[3];
} MIDIDATA;

const unsigned int batt_mv_curve[] = {4200, 4030, 3860, 3830, 3790, 3700, 3600};
const unsigned char batt_charge_curve[] = {100, 76, 52, 42, 30, 11, 0};

const int ledPin = LED_BUILTIN;

#define DEVICE_DESCRIPTION "ReaperRemote v0.0.1"

#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

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
              Serial.println("res != AT_OK");
              Serial.println(AT_ERROR_STRING);
            }
          }
        }
      }
    } // end serial available
  } // end while
}

#endif
