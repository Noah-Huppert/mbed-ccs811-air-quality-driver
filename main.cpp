#include "mbed.h"
#include "math.h"

/**
 * All the following code is original, no libraries, other than what Mbed 
 * provides, were used.
 */

I2C i2c(p9, p10);

/*
Byte reference:

index: 7   6  5  4  | 3  2  1  0
value: 128 64 32 16 | 8  4  2  1
       8   4  2  1

I2C reference:

Addresses are shifted to the left once b/c the LSB in an I2C address
is used to indicate if the transaction is a read or write. This bit
will be set by the Mbed I2C API.
*/

/**
 * Air sensor constants
 */
const int AIR_ADDR = 0x5A << 1;

const char AIR_STATUS_REG = 0x00;
const char AIR_STATUS_ERROR_MASK = 0x01;
const char AIR_STATUS_DATA_READY_MASK = 0x08;
const char AIR_STATUS_APP_VALID_MASK = 0x16;
const char AIR_STATUS_FW_MODE_MASK = 0x80;
const char AIR_STATUS_FW_MODE_BOOT = 0;
const char AIR_STATUS_FW_MODE_APP = 1;

const char AIR_MODE_REG = 0x01;
const char AIR_MODE_DRIVE_MODE_MASK = 0x70;
const char AIR_MODE_IDLE = 0x00;
const char AIR_MODE_1_SECOND = 0x01;

const char AIR_ERROR_ID_REG = 0xE0;
const char AIR_ERROR_ID_BAD_WRITE = 0x00;
const char AIR_ERROR_ID_BAD_READ = 0x01;
const char AIR_ERROR_ID_BAD_MODE = 0x02;
const char AIR_ERROR_ID_MAX_RESISTANCE = 0x03;
const char AIR_ERROR_ID_HEATER_FAULT = 0x04;
const char AIR_ERROR_ID_HEATER_SUPPLY = 0x05;

const char AIR_ALG_RESULT_DATA_REG = 0x02;

const char AIR_BOOT_APP_START_REG = 0xF4;

const uint16_t AIR_TVOC_MAX = 1187;

void die(char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    
    vprintf(fmt, args);
    printf("\r\n");
    
    va_end(args);
    
    exit(1);
}

/**
 * Air sensor status register fields.
 */
typedef struct {
    /**
     * Firmware mode, see AIR_STATUS_FW_MODE_* constants.
     */
    char fw_mode;
    
    /**
     * Indicates if the application on the sensor is valid.
     * Boolean.
     */
    char app_valid;
    
    /**
     * If an error has occured.
     * Boolean.
     */
    char error;
    
    /**
     * If a new data measurement is available.
     * Boolean.
     */
    char data_ready;
    
    /**
     * Raw bit packed status valid, useful to have here for debugging purposes.
     */
     char raw;
} air_status_t;

/**
 * Read air sensor status register into the air_status arugment.
 */
void air_read_status(air_status_t *air_status) {
    if (i2c.write(AIR_ADDR, &AIR_STATUS_REG, 1) != 0) {
        die("air: read_status: failed to select status register");
    }
    
    char raw_status;
    if (i2c.read(AIR_ADDR, &raw_status, 1) != 0) {
        die("air: read_status: failed to read air status register");
    }
    
    air_status->fw_mode = (raw_status & AIR_STATUS_FW_MODE_MASK) >> 7;
    air_status->app_valid = (raw_status & AIR_STATUS_APP_VALID_MASK) >> 4;
    air_status->data_ready = (raw_status &AIR_STATUS_DATA_READY_MASK) >> 3;
    air_status->error = raw_status & AIR_STATUS_ERROR_MASK;
    air_status->raw = raw_status;
}

/**
 * Read error ID from the air sensor.
 * Returns: Error ID
 */
char air_read_error_id() {
    if (i2c.write(AIR_ADDR, &AIR_ERROR_ID_REG, 1) != 0) {
        die("air: read_error_id: failed to select error id register");
    }
    
    char air_error_id;
    if (i2c.read(AIR_ADDR, &air_error_id, 1) != 0) {
        die("air: read_error_id: failed to read error id");
    }
    
    return air_error_id;
}

/**
 * Exits program with there is an error with the air sensor.
 */
void air_die() {
    air_status_t air_status;
    air_read_status(&air_status);
    if (air_status.error) {
        char air_error_id = air_read_error_id();
        char *str_air_error_id = NULL;
        
        switch(air_error_id) {
            case AIR_ERROR_ID_BAD_WRITE:
                str_air_error_id = "a write occurred for an invalid register address";
                break;
            case AIR_ERROR_ID_BAD_READ:
                str_air_error_id = "a read occurred for an invalid register address";
                break;
            case AIR_ERROR_ID_BAD_MODE:
                str_air_error_id = "the measurement drive mode is invalid";
                break;
            case AIR_ERROR_ID_MAX_RESISTANCE:
                str_air_error_id = "the resistance is set too high";
                break;
            case AIR_ERROR_ID_HEATER_FAULT:
                str_air_error_id = "the heater's current was not in range";
                break;
            case AIR_ERROR_ID_HEATER_SUPPLY:
                str_air_error_id = "the heater's voltage is not being applied correctly";
                break;
            default:
                str_air_error_id = "unknown error!";
                break;
        }
        
        die("air: error: %s\r\n", str_air_error_id);
    }
}

/**
 * Boot air sensor.
 * If already booted exits silently.
 */
void air_boot() {
    // Check if sensor is in a valid state to be booted
    air_status_t air_status;
    air_read_status(&air_status);
    
    // Check if already booted
    if(air_status.fw_mode == AIR_STATUS_FW_MODE_APP) {
        // Already booted
        return;
    }
    
    // Check if a valid application is loaded to be booted
    if (!air_status.app_valid) {
        die("air: boot: cannot boot, invalid app on device");
    }
    
    // Send boot command
    if (i2c.write(AIR_ADDR, &AIR_BOOT_APP_START_REG, 1) != 0) {
        die("air: boot: failed to boot");
    }
}

/**
 * Read measurement drive mode.
 * Returns: drive mode
 */
char air_read_mode() {
    if (i2c.write(AIR_ADDR, &AIR_MODE_REG, 1) != 0) {
         die("air: read_mode: failed to set drive mode to constant high freq");
    }
    
    char air_mode_buf;
    if (i2c.read(AIR_ADDR, &air_mode_buf, 1) != 0) {
        die("air: read_mode: failed to read mode register");
    }
    
    air_mode_buf = (air_mode_buf & AIR_MODE_DRIVE_MODE_MASK) >> 4;
    
    return air_mode_buf;
}

/**
 * Set measurement drive mode.
 * Previous measurement drive mode is read so new drive_mode value can be inserted into the
 * bitpacked register correctly.
 */
void air_write_mode(char drive_mode) {
    // Read current measurement mode
    if (i2c.write(AIR_ADDR, &AIR_MODE_REG, 1) != 0) {
        die("air: write_mode: failed to select measurement mode register");
    }
    
    char measurement_mode;
    if (i2c.read(AIR_ADDR, &measurement_mode, 1) != 0) {
        die("air: write_mode: failed to read measurement mode register");
    }
    
    // Bitpack new drive_mode into measurement_mode
    char write_drive_mode = drive_mode << 4;
    measurement_mode = measurement_mode & (!AIR_MODE_DRIVE_MODE_MASK);
    
    char write_measurement_mode = write_drive_mode | measurement_mode;
    
    char buf[2] = {
        AIR_MODE_REG,
        write_measurement_mode,
    };
    
    if (i2c.write(AIR_ADDR, buf, 2) != 0) {
        die("air: write_mode: failed to write mode %d", drive_mode);
    }
}

/**
 * Air sensor algorithm result data.
 */
typedef struct {
    /**
     * Equivalent calculated carbon-dioxide (eCO2) in ppm from 400 to 8192.
     */
    uint16_t eco2;
    
    /**
     * Total volume of carbon (TVOC) in ppb from 0 1187.
     */
    uint16_t tvoc;
} air_alg_result_t;

void air_read_alg_result(air_alg_result_t *air_alg_result) {
    // Read register
    if (i2c.write(AIR_ADDR, &AIR_ALG_RESULT_DATA_REG, 1) != 0) {
        die("air: read_alg_result: failed to select alg result data register");
    }
    
    char buf[4];
    if (i2c.read(AIR_ADDR, buf, 4) != 0) {
        die("air: read_alg_result: failed to read alg result data register");
    }
    
    // Unpack into air_alg_result_data_t
    air_alg_result->eco2 = (buf[0] << 8) | buf[1];
    air_alg_result->tvoc = (buf[2] << 8) | buf[3];
}

int main() {
    air_status_t air_status;
    
    // Boot air sensor
    printf("air: booting\r\n");
    air_boot();
    air_die();
    printf("air: booted\r\n");
    
    // Set measurement drive mode
    printf("air: setting measurement mode\r\n");
    air_write_mode(AIR_MODE_1_SECOND);
    air_die();
    printf("air: set measurement mode\r\n");
    
    while (1) {
        // Poll until new sample is ready
        air_read_status(&air_status);
        
        do {
            // Read status
            printf("air: polling status until data ready\r\n");
            air_read_status(&air_status);
            air_die();
            wait(0.5);
        } while (!air_status.data_ready);
    
        printf("air: data ready\r\n");
        
        air_alg_result_t air_alg_result;
        air_read_alg_result(&air_alg_result);
        
        printf("air: tvoc=%d\r\n", air_alg_result.tvoc);
    }
}
