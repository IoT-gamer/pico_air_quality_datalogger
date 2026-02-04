#ifndef SENSORS_DATALOGGER_H
#define SENSORS_DATALOGGER_H

#include <stdint.h>

typedef struct {
    // HM3301 Particulate Matter
    uint16_t pm1_0_std;
    uint16_t pm2_5_std;
    uint16_t pm10_std;
    
    // Grove Multichannel Gas V2
    uint32_t gas_no2;
    uint32_t gas_c2h5oh;
    uint32_t gas_voc;
    uint32_t gas_co;

    // SCD41 Environmental Data
    uint16_t scd_co2;       // ppm
    int32_t  scd_temp;      // milli-degrees C (divide by 1000 for float)
    int32_t  scd_hum;       // milli-RH (divide by 1000 for float)    

} air_quality_reading_t;

#endif