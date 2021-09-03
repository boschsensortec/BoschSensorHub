#include <Arduino.h>
#include <Wire.h>
#include <bhy.h>
#include <firmware/Bosch_PCB_7183_di03_BMI160-7183_di03.2.1.11696_170103.h>

#define BHY_INT_PIN 10


BHYSensor bhi160;

volatile bool intrToggled = false;
bool newOrientationData = false;
float heading, roll, pitch;
uint8_t status;

bool checkSensorStatus(void);


void orientationHandler(bhyVector data, bhyVirtualSensor type)
{
    Serial.println("new data");
    heading = data.x;
    roll = data.z;
    pitch = data.y;
    status = data.status;
    newOrientationData = true;
}


void metaHandler(bhyMetaEvent *event, bhyMetaEventType type) {
    Serial.println("hei");
}
bool waitForBhyInterrupt() {
    delay(100);
    bhyInterruptStatus intStatus;
    bhi160.getInterruptStatus(&intStatus);
    Serial.println("waiting for interrupt");
    return (intStatus.interrupt_status != 1);
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    bhi160.begin(BHY_I2C_ADDR2);

    /* Check to see if something went wrong. */
    // !checkSensorStatus();

    Serial.println("Sensor found over I2C! Product ID: 0x" + String(bhi160.productId, HEX));

    if (bhi160.loadFirmware(bhy1_fw) != 0) {
        Serial.println(F("oobs. something didnt go right."));
    }

    while (waitForBhyInterrupt());  /* Wait for meta events from boot up */
    Serial.println(F("Firmware booted"));


    /* Install a metaevent callback handler and a timestamp callback handler here if required before the first run */
    int8_t e = bhi160.installMetaCallback(BHY_META_TYPE_SELF_TEST_RESULTS, false, metaHandler);
    Serial.println(e, HEX);
    // Install a vector callback function to process the data received from the wake up Orientation sensor
    bhi160.setHostInterfaceControlBit(BHY_REQUEST_SENSOR_SELF_TEST_BIT, 1);
    bhi160.run(); /* The first run processes all boot events */

    if (bhi160.configVirtualSensor(BHY_VS_ACCELEROMETER, true, BHY_FLUSH_ALL, 25, 0, 0, 0))
    {
        //Serial.println(F("Failed to enable sensor"));
        Serial.println("Failed to enable virtual sensor (" + bhi160.getSensorName(BHY_VS_ACCELEROMETER) + "). Loaded firmware may not support requested sensor id.");
    }
    else
        Serial.println(F("Sensor enabled"));
        //Serial.println(bhi160.getSensorName(BHY_VS_ORIENTATION) + " virtual sensor enabled");

    
    int8_t ret = bhi160.installSensorCallback(BHY_VS_ACCELEROMETER, true, orientationHandler);
    Serial.println(ret, HEX);
    if (ret)
    {
        checkSensorStatus();
    }
    else
        Serial.println(F("Orientation callback installed"));



}

void loop()
{
        bhi160.run();
        Serial.println(String(heading) + "," + String(pitch) + "," + String(roll) + "," + String(status));

        
        //delay(1000);
        
}

bool checkSensorStatus(void)
{

    bhyChipStatus stats;
    bhi160.getChipStatus(&stats);
    Serial.println(F("Chip status:"));
    Serial.println(stats.ee_upload_done,HEX);
    Serial.println(stats.ee_upload_error,HEX);
    Serial.println(stats.eeprom_detected,HEX);
    Serial.println(stats.firmware_idle,HEX);
    Serial.println(stats.no_eeprom,HEX);

/*


bhyPhysicalStatus a,b,c;
bhi160.getPhysicalSensorStatus(&a,&b,&c);

Serial.println(a.flag,HEX);
Serial.println(b.flag,HEX);
Serial.println(c.flag,HEX);
*/
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