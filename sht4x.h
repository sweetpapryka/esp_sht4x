#pragma once

#include "freertos/idf_additions.h"

#include "esp_timer.h"
#include "driver/i2c_master.h"


// Edit mutex timeout values. All in milliseconds
#define SHT4X_MASTER_MUTEX_TIMEOUT				100
#define SHT4X_DEVICE_MUTEX_TIMEOUT				100
#define SHT4X_TRANSACTION_TIMEOUT				100

#define SHT4X_INTEGER_PRECISION					10000000				// (MAX 7 0's) The precision of the data in the data retrieval function that uses integers for its caluclations. (f.e no precision = 0 would return 5 but with precision of 2 would return 523)


typedef enum
{
	SHT4X_REPEAT_HIGH = 0,				// Measuring time: 8.3ms
	SHT4X_REPEAT_MEDIUM,				// 4.5ms
	SHT4X_REPEAT_LOW					// 1.6ms
} sht4x_repeatability_t;

// Sensor heater mode when measuring. Power values are given at 3.3V VDD
typedef enum
{
    SHT4X_HEATER_OFF = 0,      // Heater is off, default 
    SHT4X_HEATER_HIGH_LONG,    // High power (~200mW), 1 second pulse 
    SHT4X_HEATER_HIGH_SHORT,   // High power (~200mW), 0.1 second pulse 
    SHT4X_HEATER_MEDIUM_LONG,  // Medium power (~110mW), 1 second pulse 
    SHT4X_HEATER_MEDIUM_SHORT, // Medium power (~110mW), 0.1 second pulse 
    SHT4X_HEATER_LOW_LONG,     // Low power (~20mW), 1 second pulse 
    SHT4X_HEATER_LOW_SHORT,    // Low power (~20mW), 0.1 second pulse
} sht4x_heater_t;

// SHT4X possible addresses
typedef enum
{
	SHT4X_ADDR_1 = 0x44,
	SHT4X_ADDR_2 = 0x45,
	SHT4X_ADDR_3 = 0x46
}sht4x_scl_adress_t;

// I2C speed mode on which the device runs on
typedef enum
{
	STANDARD 		= 100000,
	FAST_MODE 		= 400000,
	FAST_MODE_PLUS 	= 1000000
}sht4x_scl_speed_t;


typedef struct i2c_master_bus
{
	// Don't touch
	i2c_master_bus_handle_t master_bus_handle;
	SemaphoreHandle_t master_bus_mutex;				// Mutex for the port access
}sht4x_i2c_master_bus_ctx_t;

typedef struct sht4x
{
	// Configure before passing descriptor to functions
	sht4x_repeatability_t repeatability; 			// The accuracy at which the device measures. High repeatability means it measures more times thus granting a more accurate result, although it takes more time complete the measurement. When heater is on repeatability is always HIGH
	sht4x_heater_t heater;							// Determines the strength of the heating and the amount of time it heats. Used to fight condensation on the sensor
	
	// Don't touch
	i2c_master_dev_handle_t dev_handle;					
	SemaphoreHandle_t master_bus_mutex;				// Place to hold the mutex for the port the device uses
	
	SemaphoreHandle_t device_access_mutex;			// Protects access to the device while it's measuring, booting from soft-reset  							 
	esp_timer_handle_t timer;						// For callback that gives the device_access_mutex after the measuring time has passed
	
	const char *name;								// Holds the name of the device. Used for debugging print messages
}sht4x_t;


/*
* Initialize I2C master bus handle and create mutex for this exact master bus
* @param master_bus_ctx[out]		Struct of the master bus context				
* @param master_bus_config			Configuration struct of the master bus
* @return esp_err_t					`ESP_OK` on success
*/
esp_err_t sht4x_i2c_master_bus_init(sht4x_i2c_master_bus_ctx_t *master_bus_ctx, i2c_master_bus_config_t master_bus_config);

/*
* Initialize I2C slave device according to its constraints
* @param master_bus_handle		Handle to the I2C master bus
* @param device_desc[out]		sht4x_t device descriptor
* @param device_addr 			SHT4x addr
* @param speed_mode  			I2C speed mode of this master and slave communication
* @param disable_ack_check		Disable ACK check. If this is set false, that means ack check is enabled, the transaction will be stopped and API returns error when nack is detected.		
* @return esp_err_t				`ESP_OK` on success
*/
esp_err_t sht4x_i2c_device_init(sht4x_i2c_master_bus_ctx_t *master_bus_ctx, sht4x_t *device_desc, const char *device_name,  sht4x_scl_adress_t device_addr, sht4x_scl_speed_t speed_mode, bool disable_ack_check);

/*
 * Soft resets the sht4x device
 * @param device_desc		device descriptor
 * @return esp_err_t		`ESP_OK` on success
 */
esp_err_t sht4x_reset_device(sht4x_t *device_desc);

/*
 * Send command for the device to start measuring temperature and humidty to later retrieve. Turn on heater if a mode is specified in the descriptor
 * @param device_desc		sht4x_t device desriptor
 * @return esp_err_t		`ESP_OK` on success
 */
esp_err_t sht4x_measure(sht4x_t *device_desc);

/*
 * I2C read to retrieve temperature and humidity data from the sensor. Data returns in integer format
 * @param device_desc			sht4x_t device desriptor
 * @param[out] temperature  	variable to put the measured temperature into
 * @param[out] humidity	  		variable to put the measured humidity into
 * @return esp_err_t			`ESP_OK` on success
 */
esp_err_t sht4x_read(sht4x_t *device_desc, int32_t *temperature, int32_t *humidity);

/*
 * I2C read to retrieve temperature and humidity data from the sensor. Data returns in float format
 * @param device_desc			sht4x_t device desriptor
 * @param[out] temperature  	variable to put the measured temperature into
 * @param[out] humidity	  		variable to put the measured humidity into
 * @return esp_err_t			`ESP_OK` on success
 */
esp_err_t sht4x_read_float(sht4x_t *device_desc, float *temperature, float *humidity);

/*
 * I2C read to get the serial number of the device 
 * @param device_desc			sht4x_t device desriptor
 * @param[out] serial_number	current devices serial number
 * @return esp_err_t			`ESP_OK` on success
*/
esp_err_t sht4x_read_serial(sht4x_t *device_desc, uint32_t *serial_number);

/*
 * Free memory from the sht4x_i2c_master_bus_ctx_t. Uninitializes the I2C port making it unusable. FREE ALL DEVICES ASSOCIATED WITH THIS PORT USING sht4x_free_device BEFORE FREEING THE PORT ITSELF
 * @param master_bus_ctx		Struct of the master bus context
 * @return esp_err_t			`ESP_OK` on success
 */
esp_err_t sht4x_free_port(sht4x_i2c_master_bus_ctx_t *master_bus_ctx);

/*
 * Free memory from the sht4x_t. Uninitialize the I2C device making it unusable.
 * @param device_desc			sht4x_t device desriptor
 * @return esp_err_t			`ESP_OK` on success
 */
esp_err_t sht4x_free_device(sht4x_t *device_desc);





































