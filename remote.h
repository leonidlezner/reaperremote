#ifndef __REMOTE_H_
#define __REMOTE_H_

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

#endif
