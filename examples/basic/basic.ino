#include <Wire.h>

#include "bhy.h"
#include "firmware\Bosch_PCB_7183_di03_BMI160-7183_di03.2.1.11696_170103.h"

#define BHY_INT_PIN 10

BHYSensor bhi160;

volatile bool intrToggled = false;

bool checkSensorStatus(void);

void bhyInterruptHandler(void)
{
    intrToggled = true;
}
void waitForBhyInterrupt(void)
{
    while (!intrToggled)
        ;
    intrToggled = false;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    if (Serial)
    {
        Serial.println("Serial working");
    }

    attachInterrupt(BHY_INT_PIN, bhyInterruptHandler, RISING);

    bhi160.begin(BHY_I2C_ADDR2);

    // Check to see if something went wrong.
    if (!checkSensorStatus())
        return;

    Serial.println("Sensor found over I2C! Product ID: 0x" + String(bhi160.productId, HEX));

    Serial.println("Uploading Firmware.");
    bhi160.loadFirmware(bhy1_fw);

    if (!checkSensorStatus())
        return;

    intrToggled = false; /* Clear interrupt status received during firmware upload */
    waitForBhyInterrupt();  /* Wait for meta events from boot up */
    Serial.println("Firmware booted");

    /* Install a metaevent callback handler and a timestamp callback handler here if required before the first run */
    bhi160.run(); /* The first run processes all boot events */

    /* Link callbacks and configure desired virtual sensors here */

    if (checkSensorStatus())
        Serial.println("All ok");
}

void loop()
{
    if (intrToggled)
    {
        intrToggled = false;
        bhi160.run();
        checkSensorStatus();
    }
}

bool checkSensorStatus(void)
{
    if (bhi160.status == BHY_OK)
        return true;

    if (bhi160.status < BHY_OK) /* All error codes are negative */
    {
        Serial.println("Error code: (" + String(bhi160.status) + "). " + bhi160.getErrorString(bhi160.status));

        return false; /* Something has gone wrong */
    }
    else /* All warning codes are positive */
    {
        Serial.println("Warning code: (" + String(bhi160.status) + ").");

        return true;
    }

    return true;
}