#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

int main() {
    VL53L0X_Dev_t dev;
    VL53L0X_DEV Dev = &dev;

    int status = 0;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    printf("VL53L0X test start\n");

    // Initialize I2C device address
    Dev->I2cDevAddr = 0x52; // Default address (7-bit address = 0x29)

    // Initialize the sensor
    status = VL53L0X_WaitDeviceBooted(Dev);
    if (status) {
        printf("Error booting device\n");
        return status;
    }

    status = VL53L0X_DataInit(Dev);
    if (status) {
        printf("DataInit error\n");
        return status;
    }

    // Reference calibration
    status = VL53L0X_StaticInit(Dev);
    if (status) {
        printf("StaticInit error\n");
        return status;
    }

    status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    if (status) {
        printf("RefCalibration error\n");
        return status;
    }

    status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    if (status) {
        printf("RefSpadManagement error\n");
        return status;
    }

    // Set device mode (single ranging)
    status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status) {
        printf("SetDeviceMode error\n");
        return status;
    }

    printf("Start single ranging measurements\n");

    while (1) {
        VL53L0X_RangingMeasurementData_t measurement;

        status = VL53L0X_PerformSingleRangingMeasurement(Dev, &measurement);
        if (status) {
            printf("Measurement error\n");
        } else {
            printf("Distance = %d mm\n", measurement.RangeMilliMeter);
        }

        usleep(500000); // 500ms delay
    }

    return 0;
}
