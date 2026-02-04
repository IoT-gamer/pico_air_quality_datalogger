#include "sd_logger.h"
#include <stdio.h>
#include "hw_config.h"
#include "pico/util/datetime.h"
#include "pico/aon_timer.h" 
#include <time.h>
#include "f_util.h" 
#include "ff.h" 

// *** FALLBACK: If pico/util/datetime.h failed to define datetime_t ***
// This block ensures datetime_t is known to the C compiler.
#ifndef datetime_t
typedef struct {
    int16_t year;
    int8_t month;
    int8_t day;
    int8_t dotw;
    int8_t hour;
    int8_t min;
    int8_t sec;
} datetime_t;
#endif

extern bool ble_server_is_rtc_synced(void);

// --- SD Card Globals ---
static FATFS fs; 
static bool sd_mounted = false; 

bool sd_logger_init(void) {
    printf("Mounting SD card...\n");
    FRESULT fr = f_mount(&fs, "", 1); 
    if (FR_OK != fr) {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr); 
        sd_mounted = false; 
    } else {
        printf("SD card mounted successfully.\n");
        sd_mounted = true; 
    }
    return sd_mounted;
}

void sd_logger_log_reading(air_quality_reading_t *reading) {
    if (!sd_mounted) {
        printf("SD card not mounted. Skipping log.\n");
        return; 
    }

    // Check if the RTC has been synced before proceeding with time-dependent operations
    if (!ble_server_is_rtc_synced()) {
        printf("RTC has not been synced by BLE client. Skipping log entry.\n");
        return;
    }

    // --- Get and format timestamp ---
    datetime_t t;
    char timestamp_buf[32]; // Buffer for "YYYY-MM-DDTHH:MM:SS" 
    char filename_buf[32];  // Buffer for "YYYY-MM-DD.txt"
    
    // Get time using the unified AON Timer API (as struct tm)
    struct tm tm_struct;
    if (!aon_timer_get_time_calendar(&tm_struct)) {
        printf("Failed to get AON timer time (even if synced). Skipping log.\n");
        return;
    }

    // CRITICAL VALIDITY CHECK: The RTC is set to 1900/1970 
    // before the mobile app syncs it. FATFS requires >= 1980.
    if (tm_struct.tm_year < (1980 - 1900)) { 
        printf("RTC not synced (Year < 1980). Skipping log.\n");
        // We need a way to check if RTC is synced
        
    // CRITICAL CHECK: Only proceed if RTC is confirmed synced
    if (!ble_server_is_rtc_synced()) {
        printf("RTC has not been synced by BLE client. Skipping log entry.\n");
        return;
    }
        // If ble_server says it's synced but the year is wrong, there's another issue.
    }

    // Convert struct tm (tm_struct) back to Pico's datetime_t (t)
    // The call to tm_to_datetime(&tm_struct, &t); is replaced by the lines below.
    t.year = tm_struct.tm_year + 1900;
    t.month = tm_struct.tm_mon + 1;
    t.day = tm_struct.tm_mday;
    t.hour = tm_struct.tm_hour;
    t.min = tm_struct.tm_min;
    t.sec = tm_struct.tm_sec;
    t.dotw = tm_struct.tm_wday; // Weekday is optional but included for completeness
    
    // Format as ISO 8601 for the log line
    snprintf(timestamp_buf, sizeof(timestamp_buf),
             "%04d-%02d-%02dT%02d:%02d:%02d",
             t.year, t.month, t.day, t.hour, t.min, t.sec);
    
    // Format the daily filename
    snprintf(filename_buf, sizeof(filename_buf),
             "%04d-%02d-%02d.txt",
             t.year, t.month, t.day);
    // ------------------------------------------

    FIL fil;
    
    // Use the dynamic filename_buf
    FRESULT fr = f_open(&fil, filename_buf, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr) {
        // Use filename_buf in the error message
        printf("f_open(%s) error: %s (%d)\n", filename_buf, FRESULT_str(fr), fr);
        return; 
    }

    // --- Write timestamp + data as a CSV-like string ---
    int chars_written = f_printf(&fil, "%s,PM1_0:%u,PM2_5:%u,PM10:%u,NO2:%lu,Eth:%lu,VOC:%lu,CO:%lu,CO2:%u,T:%ld,H:%ld\n", 
        timestamp_buf,
        reading->pm1_0_std,
        reading->pm2_5_std,
        reading->pm10_std,
        reading->gas_no2,
        reading->gas_c2h5oh,
        reading->gas_voc,
        reading->gas_co,
        reading->scd_co2,
        reading->scd_temp,
        reading->scd_hum
    );

    if (chars_written < 0) {
        printf("f_printf failed\n");
    }

    // Close the file (this also flushes the write buffer)
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    } else {
        // Use filename_buf in the success message
        printf("Successfully logged reading to %s\n", filename_buf);
    }
}