#include <Wire.h>

#include "bhy.h"
#include "firmware\Bosch_PCB_7183_di03_BMI160_BMM150-7183_di03.2.1.11696_170103.h"

#define BHY_INT_PIN 10

BHYSensor bhi160;

volatile bool intrToggled = false;
bool newOrientationData = false;
float heading, roll, pitch;
uint8_t status;

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
void orientationHandler(bhyVector data, bhyVirtualSensor type)
{
    heading = data.x;
    roll = data.z;
    pitch = data.y;
    status = data.status;
    newOrientationData = true;
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

    /* Check to see if something went wrong. */
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
    
    /* Install a vector callback function to process the data received from the wake up Orientation sensor */
    if (bhi160.installSensorCallback(BHY_VS_ORIENTATION, true, orientationHandler))
    {
        checkSensorStatus();

        return;
    }
    else
        Serial.println("Orientation callback installed");

    /* Enable the Orientation virtual sensor that gives you the heading, roll, pitch
       based of data from the accelerometer, gyroscope and magnetometer.
       The sensor is set into wake up mode so as to interrupt the host when a new sample is available
       Additionally, the FIFO buffer of the sensor is flushed for all previous data
       The maximum report latency of the sensor sample, the sensitivity and the dynamic range
       are set to 0
     */
    if (bhi160.configVirtualSensor(BHY_VS_ORIENTATION, true, BHY_FLUSH_ALL, 200, 0, 0, 0))
    {
        Serial.println("Failed to enable virtual sensor (" + bhi160.getSensorName(
                           BHY_VS_ORIENTATION) + "). Loaded firmware may not support requested sensor id.");
    }
    else
        Serial.println(bhi160.getSensorName(BHY_VS_ORIENTATION) + " virtual sensor enabled");

}

void loop()
{
    if (intrToggled)
    {
        intrToggled = false;
        bhi160.run();
        checkSensorStatus();
        if (newOrientationData)
        {
            /* Can also be viewed using the plotter */
            Serial.println(String(heading) + "," + String(pitch) + "," + String(roll) + "," + String(status));
            newOrientationData = false;
        }
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