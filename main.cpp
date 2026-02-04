#include <stdio.h>
#include <string.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/aon_timer.h"
#include "pico/time.h"
#include <time.h> 

// --- Project Modules (C files must be declared extern "C") ---
extern "C" {
    #include "ble_server.h"
    #include "sd_logger.h"
    #include "grove_multigas_v2.h"
    #include "scd4x_i2c.h"
    #include "sensirion_i2c_hal.h"
}
#include "sensors_datalogger.h"

// --- Hardware Modules ---
#include "hm3301.h" // HM3301 C++ Driver header

// --- Configuration ---
#define LED_QUICK_FLASH_DELAY_MS 100 
#define LED_SLOW_FLASH_DELAY_MS 1000 

#define SYNC_TIMEOUT_MS 30000  // 30 seconds for initial RTC sync
#define LOG_INTERVAL_MS (15 * 60 * 1000) // 15 minutes
#define LIVE_UPDATE_INTERVAL_MS 5000     // 5 seconds for live update

// Define HM3301 I2C pins and port
#define I2C_PORT i2c0
#define SDA_PIN 8
#define SCL_PIN 9

// --- Global State ---
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_timer_source_t heartbeat; 
static btstack_timer_source_t server_advertisement_timer;
static btstack_timer_source_t live_update_timer;

HM3301 *sensor_ptr = NULL; // Global pointer to HM3301 sensor object

// Forward declaration from ble_server.c/h
extern void ble_server_start_advertising(void); 
extern void ble_server_stop_advertising(void); 

// --- Function Declarations ---
static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void heartbeat_handler(struct btstack_timer_source *ts);
static void server_timeout_handler(struct btstack_timer_source *ts);
static void enter_server_mode(void);
static void run_sensor_read_and_log(void);
static void live_update_handler(struct btstack_timer_source *ts);

/**
 * @brief Performs the sensor read and BLE notification.
 * (Runs every 5 seconds while connected)
 */
static void run_sensor_read_and_notify(void) {
    // Read & Notify PM2.5
    if (sensor_ptr != NULL) {
        uint8_t raw_buffer[30];
        HM330XErrorCode err = sensor_ptr->read_sensor_value(raw_buffer, 29);
        if (err == HM330X_NO_ERROR) {
            HM3301::Data val = HM3301::parse_data(raw_buffer);
            ble_server_notify_pm25(val.pm2_5_std);
        }
    }

    // Read & Notify Gas Values
    // Read the latest raw values from the Multichannel Gas V2 sensor
    uint32_t val_no2    = multigas_v2_read_no2();
    uint32_t val_c2h5oh = multigas_v2_read_c2h5oh();
    uint32_t val_voc    = multigas_v2_read_voc();
    uint32_t val_co     = multigas_v2_read_co();

    // Send the notification
    ble_server_notify_gas(val_no2, val_c2h5oh, val_voc, val_co);

    // SCD41 Live Update
    // We check if data is ready to avoid blocking the BLE stack for too long
    bool data_ready = false;
    scd4x_get_data_ready_status(&data_ready);
    
    if (data_ready) {
        uint16_t co2;
        int32_t temp, hum;
        if (scd4x_read_measurement(&co2, &temp, &hum) == 0) {
            ble_server_notify_env(co2, temp, hum);
        }
    }

}

/**
 * @brief Performs the sensor read, data parsing, and LOGGING.
 * (Runs every 15 mins by default)
 */
static void run_sensor_read_and_log(void) {
    // Check HM3301 Object
    if (sensor_ptr == NULL) {
        printf("Sensor pointer is NULL. Skipping log.\n");
        return;
    }

    // Initialize reading struct (Zero out to ensure clean data)
    air_quality_reading_t current_reading = {0};
    uint8_t raw_buffer[30];

    // -----------------------------------------------------
    // Read HM3301 (Particulate Matter)
    // -----------------------------------------------------
    HM330XErrorCode err = sensor_ptr->read_sensor_value(raw_buffer, 29);
    
    if (err == HM330X_NO_ERROR) {
        // Parse raw bytes into HM3301::Data struct
        HM3301::Data val = HM3301::parse_data(raw_buffer);

        // Populate PM data
        current_reading.pm1_0_std = val.pm1_0_std;
        current_reading.pm2_5_std = val.pm2_5_std;
        current_reading.pm10_std  = val.pm10_std;
    } else {
        printf("HM3301 read failed with error: %d\n", err);
        // We continue executing so we can still log the Gas data if available
    }

    // -----------------------------------------------------
    // Read Grove Multichannel Gas V2
    // -----------------------------------------------------
    // These functions return uint32_t raw ADC values
    current_reading.gas_no2    = multigas_v2_read_no2();
    current_reading.gas_c2h5oh = multigas_v2_read_c2h5oh();
    current_reading.gas_voc    = multigas_v2_read_voc();
    current_reading.gas_co     = multigas_v2_read_co();

    // -----------------------------------------------------
    // Read SCD41 Environmental Data
    // -----------------------------------------------------
    // For logging, we can afford to block slightly or wait for ready
    bool data_ready = false;
    scd4x_get_data_ready_status(&data_ready);
    if (data_ready) {
        scd4x_read_measurement(&current_reading.scd_co2, 
                               &current_reading.scd_temp, 
                               &current_reading.scd_hum);
    } else {
        printf("SCD4x data not ready for log.\n");
    }

    // -----------------------------------------------------
    // Console Debug Output
    // -----------------------------------------------------
    printf("Log Cycle -> PM2.5: %u | NO2: %lu | Eth: %lu | VOC: %lu | CO: %lu | CO2: %u | T: %ld | H: %ld\n",
           current_reading.pm2_5_std,
           current_reading.gas_no2,
           current_reading.gas_c2h5oh,
           current_reading.gas_voc,
           current_reading.gas_co,
           current_reading.scd_co2,
           current_reading.scd_temp,
           current_reading.scd_hum);

    // -----------------------------------------------------
    // Log Combined Data to SD Card
    // -----------------------------------------------------
    sd_logger_log_reading(&current_reading);
}

/**
 * @brief Handles the 5-second timer cycle while connected.
 */
static void live_update_handler(struct btstack_timer_source *ts) {
    
    // CALL THE NEW LIGHTWEIGHT FUNCTION
    run_sensor_read_and_notify();
    
    // Reschedule the timer for the next run (5000ms)
    if (ble_server_get_con_handle() != HCI_CON_HANDLE_INVALID) {
        btstack_run_loop_set_timer(ts, LIVE_UPDATE_INTERVAL_MS);
        btstack_run_loop_add_timer(ts); 
    }
}

/**
 * @brief Enters the default "server" state.
 * Advertises and sets a timer for the next action.
 */ 
static void enter_server_mode(void){
    // First, always remove the timer.
    btstack_run_loop_remove_timer(&server_advertisement_timer); 
    
    // Check if RTC is synced and set timer accordingly
    bool rtc_synced = ble_server_is_rtc_synced(); // Assume this is from ble_server.h
    uint32_t timeout_ms = rtc_synced ? LOG_INTERVAL_MS : SYNC_TIMEOUT_MS;
    
    if (rtc_synced) {
        printf("Entering server mode. Waiting %lu mins for next log cycle...\n", LOG_INTERVAL_MS / 60000); 
    } else {
        printf("Entering server mode. Advertising for RTC sync (%lus)...\n", SYNC_TIMEOUT_MS / 1000); 
    }    

    ble_server_start_advertising();
    
    // Now it is safe to set up and add the timer
    btstack_run_loop_set_timer_handler(&server_advertisement_timer, server_timeout_handler);
    btstack_run_loop_set_timer(&server_advertisement_timer, timeout_ms);
    btstack_run_loop_add_timer(&server_advertisement_timer); 
}

/**
 * @brief Handles the server mode timeout.
 * Stops advertising and either starts sensor read or re-enters server mode.
 */
static void server_timeout_handler(struct btstack_timer_source *ts) {
    UNUSED(ts);
    
    ble_server_stop_advertising();
    if (ble_server_is_rtc_synced()) {
        printf("Server mode timed out, RTC is synced. Starting sensor read.\n");
        
        // This runs every 15 minutes
        run_sensor_read_and_log(); // <--- This function handles the SD logging
 
        // Return to server mode...
        enter_server_mode();
    } else {
        // ... restart server mode ...
        enter_server_mode();
    }
}

/**
 * @brief HCI Event Handler for BLE events.
 */
static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size); 
    UNUSED(channel); 
    bd_addr_t local_addr;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet); 

    switch(event_type){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                gap_local_bd_addr(local_addr);
                printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr)); 
                // Start in server mode
                enter_server_mode();
            }
            break;
            
        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
              
                    printf("Client connected to our server. Staying in server mode.\n"); 
                    btstack_run_loop_remove_timer(&server_advertisement_timer); 
                    ble_server_handle_hci_event(packet_type, channel, packet, size); 
                    
                    // START LIVE UPDATE TIMER: 1ms delay for immediate first run.
                    // ENSURE TIMER IS REMOVED FIRST
                    btstack_run_loop_remove_timer(&live_update_timer); // Defensive removal
                    btstack_run_loop_set_timer_handler(&live_update_timer, live_update_handler);
                    btstack_run_loop_set_timer(&live_update_timer, 1); 
                    btstack_run_loop_add_timer(&live_update_timer);
                    
                    break;
                default:
                    break;
            }
            break;
            
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            { 
                hci_con_handle_t disconnected_handle = hci_event_disconnection_complete_get_connection_handle(packet);

                // Handle server connection disconnect
                if (ble_server_get_con_handle() == disconnected_handle){
                    ble_server_set_con_handle(HCI_CON_HANDLE_INVALID);
                    printf("Client disconnected from our server.\n"); 
                    
                    // STOP LIVE UPDATE TIMER
                    btstack_run_loop_remove_timer(&live_update_timer);
                }
                
                // Now, if we are completely disconnected, re-enter server mode
                if (ble_server_get_con_handle() == HCI_CON_HANDLE_INVALID)
                {
                   
                    printf("All connections closed. Re-entering server mode.\n");
                    enter_server_mode(); // Go back to advertising
                }
            }
            break;
        default:
            break;
    }
}

/**
 * @brief Heartbeat LED Handler.
 * Flashes the LED at different rates based on connection status.
 */
static void heartbeat_handler(struct btstack_timer_source *ts) {
    static bool quick_flash;
    static bool led_on = true; 

    led_on = !led_on;
    // Check if we are connected to the server client (e.g., a phone)
    bool is_connected = ble_server_get_con_handle() != HCI_CON_HANDLE_INVALID;

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    if (is_connected && led_on) {
        quick_flash = !quick_flash;
    } else if (!is_connected) {
        quick_flash = false;
    }

    // Restart timer
    btstack_run_loop_set_timer(ts, (led_on || quick_flash) ? LED_QUICK_FLASH_DELAY_MS : LED_SLOW_FLASH_DELAY_MS); 
    btstack_run_loop_add_timer(ts);
}


int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for stdio


    // Instantiate and Initialize HM3301 Sensor
    sensor_ptr = new HM3301(I2C_PORT);

    // Initialize the I2C port and the sensor
    HM330XErrorCode init_err = sensor_ptr->init(SDA_PIN, SCL_PIN);
    
    if (init_err != HM330X_NO_ERROR) {
        printf("Failed to initialize HM3301: %d\n", init_err);
        // Maybe hang here or continue with logging disabled
    } else {
        printf("HM3301 Initialized successfully on I2C0 (SDA:%d, SCL:%d)\n", SDA_PIN, SCL_PIN); 
    }
    
    // Initialize Multichannel Gas Sensor V2 on the SAME I2C port
    // Using default address 0x08
    if (!multigas_v2_init(I2C_PORT, MULTIGAS_V2_DEFAULT_ADDR)) {
        printf("WARNING: Gas Sensor not found! check wiring.\n");
        // We don't exit here so that PM logging can still continue if gas fails
    } else {
        printf("Gas Sensor V2 Initialized.\n");
        
        // Optional: The sensor needs a warm-up time. 
        // You might want to discard the first few readings.
    }


    // --- Initialize SCD41 Sensor ---
    printf("Initializing SCD41 Sensor...\n");
    
    // Initialize HAL
    sensirion_i2c_hal_init();
    scd4x_init(SCD41_I2C_ADDR_62); 

    // Stop Periodic Measurement (Crucial startup step)
    printf("Sending SCD41 Stop...");
    int16_t err = scd4x_stop_periodic_measurement();
    if (err == 0) {
        printf("OK\n");
    } else {
        printf("Failed (Err: %d). Trying WakeUp...\n", err);
        scd4x_wake_up(); // Try to wake it up
    }
    
    sleep_ms(500);

    // Get Serial Number (Confirms bidirectional communication)
    uint16_t serial_number[3]; // Declare an array of size 3
    err = scd4x_get_serial_number(serial_number, 3);
    if (err == 0) {
        printf("SCD41 Connected! Serial: 0x%04x%04x%04x\n", 
               serial_number[0], serial_number[1], serial_number[2]);
    } else {
        printf("SCD41 Read Serial Failed (Err: %d). Check Wiring!\n", err);
    }

    // Start Measurement
    printf("Starting Measurements...");
    err = scd4x_start_periodic_measurement();
    if (err == 0) {
        printf("OK. Waiting 5s for first data...\n");
    } else {
        printf("Failed to start (Err: %d)\n", err);
    }

    // --- Initialize RTC / POWMAN Timer using AON API ---
    printf("Initializing AON Timer Abstraction...\n");
    
    // Start the timer. We don't have an initial struct tm to pass,
    // so we use the simpler start-with-time-of-day variant, 
    // which effectively initializes the timer with a zero or arbitrary time, 
    // waiting for the BLE sync to set the correct time.
    aon_timer_start_with_timeofday(); 
    
    printf("AON Timer initialized. Waiting for time sync from app...\n");
    // ------------------------------------

    printf("--- Pico W HM3301 Datalogger ---\n");
    
    // --- Initialize Modules ---
    sd_logger_init(); // Needs to be called before any logging
    // -------------------------

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        // Clean up sensor object before exit
        if (sensor_ptr) delete sensor_ptr;
        return -1; 
    }

    cyw43_arch_disable_sta_mode(); // Explicitly disable station mode (Wi-Fi)

    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    
    // Initialize BLE Server (which initializes ATT server)
    ble_server_init(hci_event_handler); 

    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration); 
    
    // Set up LED flashing timer
    heartbeat.process = &heartbeat_handler; 
    btstack_run_loop_set_timer(&heartbeat, LED_SLOW_FLASH_DELAY_MS);
    btstack_run_loop_add_timer(&heartbeat); 

    hci_power_control(HCI_POWER_ON);
    
    btstack_run_loop_execute();

    // Clean up sensor object when run loop exits
    if (sensor_ptr) delete sensor_ptr;
    return 0; 
}