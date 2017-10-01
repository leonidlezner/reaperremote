#include <Arduino.h>
#include "Feather.h"

unsigned int getBatteryVoltage()
{
    float measuredvbat = analogRead(VBATPIN);

    // Measured by a voltage divider
    measuredvbat *= 2;

    // Reference voltage
    measuredvbat *= 3.3;

    // 10 bit ADC
    measuredvbat /= 1.024;

    return measuredvbat;
}

unsigned char chargeLookup(const unsigned int *mv_curve, const unsigned char *charge_curve, unsigned int mv_in)
{
    unsigned char charge = 0;

    if (mv_in > mv_curve[0])
    {
        return charge_curve[0];
    }

    while (*charge_curve > 0)
    {
        int vp1 = *mv_curve;
        int vp2 = *(mv_curve + 1);
        int chp1 = *charge_curve;
        int chp2 = *(charge_curve + 1);

        if (mv_in <= vp1 && mv_in > vp2)
        {
            float ratio = 1.0 * (chp2 - chp1) / (vp2 - vp1);
            float offset = chp1 - vp1 * ratio;

            charge = (unsigned int)(offset + ratio * mv_in);

            return charge;
        }

        charge_curve++;
        mv_curve++;
    }

    return charge;
}
