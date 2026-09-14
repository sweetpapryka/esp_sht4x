#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sht4x.h"
#include "freertos/idf_additions.h"


#define CMD_LENGTH					1
#define CMD_SOFT_RESET				0x94
#define CMD_SERIAL           	 	0x89
#define CMD_MEAS_HIGH        	 	0xfd
#define CMD_MEAS_MED          		0xf6
#define CMD_MEAS_LOW         	 	0xe0
#define CMD_MEAS_H_HIGH_LONG  		0x39
#define CMD_MEAS_H_HIGH_SHORT 		0x32
#define CMD_MEAS_H_MED_LONG   		0x2f
#define CMD_MEAS_H_MED_SHORT  		0x24
#define CMD_MEAS_H_LOW_LONG   		0x1e
#define CMD_MEAS_H_LOW_SHORT  		0x15

#define DATA_READ_LENGTH			6				

#define COMPARE_VAL(x, y) do { if (x != y) ret = ESP_FAIL; } while (0)


// Used together with device_access_mutex in sht4x_t to deny access for specific periods when the sensor is measuring, soft-resetting (IN MICROSECONDS)
typedef enum
{
	SOFT_RESET_TIMEOFF			= 1000,
	LOW_REPEAT_TIMEOFF			= 1600,
	MEDIUM_REPEAT_TIMEOFF		= 4500,
	HIGH_REPEAT_TIMEOFF			= 8300
}sht4x_access_timeoff_t;


static const char TAG[] = "I2C_SHT4X";


// Crop the humidity if It's above 100 or below 0. Uses integers only
static inline int32_t crop_humidity(uint16_t humidity_data)
{
	// Calculate humidity only using integers. Multiply by SHT4X_INTEGER_PRECISION because needs precision a float would have
	int32_t humidity = (-6 * SHT4X_INTEGER_PRECISION + ((int64_t)125 * SHT4X_INTEGER_PRECISION * humidity_data) / 65535);
	// Humidity can be above 100 or below 0. Remove that
	if (humidity > 100 * SHT4X_INTEGER_PRECISION)
	{
		return (100 * SHT4X_INTEGER_PRECISION);
	}
	else if (humidity < 0)
	{
		return 0;
	}
	else
	{
		return humidity;
	}
}

// Crop the humidity if It's above 100 or below 0.
static inline float crop_humidity_float(uint16_t humidity_data)
{
	// Calculate humidity 
	float humidity = (-6 + (125.0f * humidity_data / 65535));
	// Humidity can be above 100 or below 0. Remove that
	if (humidity > 100)
	{
		return 100;
	}
	else if (humidity < 0)
	{
		return 0;
	}
	else
	{
		return humidity;
	}
}

// CRC check for data integrity
static uint8_t crc_check(const uint8_t *data)
{
    uint8_t crc = 0xFF;                    
    for (uint8_t i = 0; i < 2; i++) {
        crc ^= *(data + i);                    
        for (int b = 0; b < 8; b++) {      
            crc = (crc & 0x80)
                  ? (crc << 1) ^ 0x31       
                  : (crc << 1);             
        }
    }
    return crc;                            
}

// Get the time it will take (in milliseconds) to completele the measurement based on repeatability and heater choices. The time will be used to restrict access to the device
static inline sht4x_access_timeoff_t get_access_restrict_time(sht4x_t *device_desc)
{
	if (device_desc->heater == SHT4X_HEATER_OFF)
	{
		switch(device_desc->repeatability)
		{
			case SHT4X_REPEAT_HIGH:
				return HIGH_REPEAT_TIMEOFF;
			case SHT4X_REPEAT_MEDIUM:
				return MEDIUM_REPEAT_TIMEOFF;
			case SHT4X_REPEAT_LOW:
				return LOW_REPEAT_TIMEOFF;
		}
	}
	else
	{
		return HIGH_REPEAT_TIMEOFF;
	}
	
	// Non-reachable. For compilers happiness
	return HIGH_REPEAT_TIMEOFF;	
}

// Get command based on repeatability and heater settings
static inline uint8_t get_cmd(sht4x_t *device_desc)
{
		switch (device_desc->heater)
		{
			case SHT4X_HEATER_OFF:
				switch (device_desc->repeatability)
				{
					case SHT4X_REPEAT_HIGH:
						return CMD_MEAS_HIGH;
					case SHT4X_REPEAT_MEDIUM:
						return CMD_MEAS_MED;
					case SHT4X_REPEAT_LOW:
						return CMD_MEAS_LOW;
				}
				break;
			case SHT4X_HEATER_HIGH_LONG:
				return CMD_MEAS_H_HIGH_LONG;
			case SHT4X_HEATER_HIGH_SHORT:
				return CMD_MEAS_H_HIGH_SHORT;
			case SHT4X_HEATER_MEDIUM_LONG:
				return CMD_MEAS_H_MED_LONG;
			case SHT4X_HEATER_MEDIUM_SHORT:
				return CMD_MEAS_H_MED_SHORT;
			case SHT4X_HEATER_LOW_LONG:
				return CMD_MEAS_H_LOW_LONG;
			case SHT4X_HEATER_LOW_SHORT:
				return CMD_MEAS_H_LOW_SHORT;
		}
		
		// Non-reachable. For compilers happiness
		return SHT4X_REPEAT_HIGH;
}

// Callback used in esp_timer_create
static void restore_device_access(void *arg)
{
	sht4x_t *device = (sht4x_t* ) arg;
	xSemaphoreGive(device->device_access_mutex);
}

esp_err_t sht4x_i2c_master_bus_init(sht4x_i2c_master_bus_ctx_t *master_bus_ctx, i2c_master_bus_config_t master_bus_config)
{
	// esp_err_t for ESP error handling macros
	esp_err_t ret = ESP_OK;
	
	// Make sure everything is zero'd out for safety
	memset(master_bus_ctx, 0, sizeof(sht4x_i2c_master_bus_ctx_t));

	ret = i2c_new_master_bus(&master_bus_config, &(master_bus_ctx->master_bus_handle));
	ESP_RETURN_ON_ERROR(ret, TAG, "I2C PORT %d INIT FAILED", master_bus_config.i2c_port);
	
	// Create initialized ports mutex for access restriction while in use
	master_bus_ctx->master_bus_mutex = xSemaphoreCreateMutex();
	
	return ret;
}

esp_err_t sht4x_i2c_device_init(sht4x_i2c_master_bus_ctx_t *master_bus_ctx, sht4x_t *device_desc, const char *device_name,  sht4x_scl_adress_t device_addr, sht4x_scl_speed_t speed_mode, bool disable_ack_check)
{
	// esp_err_t for ESP error handling macros
	esp_err_t ret = ESP_OK;
	
	// Make sure everything is zero'd out for safety
	memset(device_desc, 0, sizeof(sht4x_t));
	
	// Append device name to device descriptor to be able to use it instantly
	if(device_name != NULL)
	{
		device_desc->name = device_name;
	}
	else
	{
		device_desc->name = "";	
	}
	
	// Initialize SHT4X device on the given I2C port
	i2c_device_config_t dev_config = {
		.dev_addr_length 			= I2C_ADDR_BIT_LEN_7,
		.device_address				= device_addr,
		.scl_speed_hz				= speed_mode,
		.scl_wait_us				= 0,
		.flags.disable_ack_check 	= disable_ack_check 
	};
	ret = i2c_master_bus_add_device(master_bus_ctx->master_bus_handle, &dev_config, &(device_desc->dev_handle));
	ESP_RETURN_ON_ERROR(ret, TAG, "%s, SHT4X I2C DEVICE INIT FAILED", device_desc->name);
	
	// Get port mutex on device descriptor for easier access and create binary semaphore for device access
	device_desc->master_bus_mutex = master_bus_ctx->master_bus_mutex;
	device_desc->device_access_mutex = xSemaphoreCreateBinary();
	xSemaphoreGive(device_desc->device_access_mutex);
	
	// Create timer which callbacks to give the device access mutex back
	const esp_timer_create_args_t timer_args = {
		.name =  "sht4x_restore_access",
		.dispatch_method = ESP_TIMER_TASK,
		.callback = restore_device_access,
		.arg = device_desc
	};
	ret = esp_timer_create(&timer_args, &(device_desc->timer));
	ESP_RETURN_ON_ERROR(ret, TAG, "%s, SHT4X DEVICE TIMER CREATION FOR ACCESS MUTEX MANAGMENT FAILED", device_desc->name);
	
	ESP_LOGI(TAG, "%s, SHT4X device initialized successfully", device_desc->name);	
	return ret;
}

esp_err_t sht4x_reset_device(sht4x_t *device_desc)
{
	// esp_err_t for ESP error handling macros
	esp_err_t ret = ESP_OK;
	
	// Take current device semaphore. Return ESP_ERR_TIMEOUT if timeout
	if(!xSemaphoreTake(device_desc->device_access_mutex, pdMS_TO_TICKS(SHT4X_DEVICE_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, DEVICE ACCESS SEMAPHORE TIMEOUT ON RESET COMMAND", device_desc->name);
	}
	
	// Take port mutex which the current device is on. Send the reset command
	const uint8_t cmd = CMD_SOFT_RESET;
	if(!xSemaphoreTake(device_desc->master_bus_mutex, pdMS_TO_TICKS(SHT4X_MASTER_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, MASTER BUS SEMAPHORE TIMEOUT ON RESET COMMAND", device_desc->name);
	}	
	ret = i2c_master_transmit(device_desc->dev_handle, &cmd, CMD_LENGTH, SHT4X_TRANSACTION_TIMEOUT);
	ESP_GOTO_ON_ERROR(ret, cleanup_master_bus, TAG, "%s, I2C SOFT-RESET CMD TRANSMISSION FAILED", device_desc->name);
	xSemaphoreGive(device_desc->master_bus_mutex);
	
	ESP_LOGI(TAG, "%s, SHT4X device (soft) reset command sent", device_desc->name);
	
	// Create timer for callback which returns access (gives device access mutex back) to the device after a safe period has elapsed
	ret = esp_timer_start_once(device_desc->timer, SOFT_RESET_TIMEOFF);
	ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "%s, FAILED TO START TIMER FOR DEVICE ACCESS MUTEX RESTORE (ON SOFT-RESET). DON'T ACCESS DEVICE FOR ATLEAST %d SECOND(S)", device_desc->name, SOFT_RESET_TIMEOFF);
	
	return ret;
	
	cleanup_master_bus:
	xSemaphoreGive(device_desc->master_bus_mutex);

	cleanup:
	xSemaphoreGive(device_desc->device_access_mutex);
	return ret;	
}

esp_err_t sht4x_measure(sht4x_t *device_desc)
{
	// esp_err_t for ESP error handling macros
	esp_err_t ret = ESP_OK;
	
	// Take current device semaphore. Return ESP_ERR_TIMEOUT if timeout
	if(!xSemaphoreTake(device_desc->device_access_mutex, pdMS_TO_TICKS(SHT4X_DEVICE_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, DEVICE ACCESS SEMAPHORE TIMEOUT ON MEASURE COMMAND", device_desc->name);
	}
	
	// Take port mutex which the current device is on. Send the measure command
	uint8_t cmd = get_cmd(device_desc);
	if(!xSemaphoreTake(device_desc->master_bus_mutex, pdMS_TO_TICKS(SHT4X_MASTER_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, MASTER BUS SEMAPHORE TIMEOUT ON MEASURE COMMAND", device_desc->name);
	}
	ret = i2c_master_transmit(device_desc->dev_handle, &cmd, CMD_LENGTH, SHT4X_TRANSACTION_TIMEOUT);
	ESP_GOTO_ON_ERROR(ret, cleanup_master_bus, TAG, "%s, I2C MEASURE CMD TRANSMISSION FAILED", device_desc->name);
	xSemaphoreGive(device_desc->master_bus_mutex);
	
	ESP_LOGI(TAG, "%s, SHT4X device measurement command sent", device_desc->name);
	
	// Create timer for callback which returns access (gives device access mutex back) to the device after a safe period has elapsed
	sht4x_access_timeoff_t delay = get_access_restrict_time(device_desc);
	ret = esp_timer_start_once(device_desc->timer, delay);
	ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "%s, FAILED TO START TIMER FOR DEVICE ACCESS MUTEX RESTORE (ON MEASURE). DON'T ACCESS DEVICE FOR ATLEAST %d SECOND(S)", device_desc->name, delay);
	
	return ret;
	
	cleanup_master_bus:
	xSemaphoreGive(device_desc->master_bus_mutex);
	
	cleanup:
	xSemaphoreGive(device_desc->device_access_mutex);
	return ret;	
}

esp_err_t sht4x_read(sht4x_t *device_desc, int32_t *temperature, int32_t *humidity)
{
	// esp_err_t for ESP error handling macros
	esp_err_t ret = ESP_OK;
	
	// Take current device semaphore. Return ESP_ERR_TIMEOUT if timeout
	if(!xSemaphoreTake(device_desc->device_access_mutex, pdMS_TO_TICKS(SHT4X_DEVICE_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, DEVICE ACCESS SEMAPHORE TIMEOUT ON READ (INTEGER)", device_desc->name);
	}
	
	// Take port mutex which the current device is on. Send I2C read
	uint8_t read_buffer[DATA_READ_LENGTH] = {0};
	if(!xSemaphoreTake(device_desc->master_bus_mutex, pdMS_TO_TICKS(SHT4X_MASTER_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, MASTER BUS SEMAPHORE TIMEOUT ON READ (INTEGER)", device_desc->name);
	}
	ret = i2c_master_receive(device_desc->dev_handle, read_buffer, DATA_READ_LENGTH, SHT4X_TRANSACTION_TIMEOUT);
	ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "%s, I2C TEMP/HUMID READ FAILED", device_desc->name);
	xSemaphoreGive(device_desc->master_bus_mutex);
	
	
	
	// Check data validity via CRC
	// CRC check temperatues data 
	uint8_t crc = crc_check(&read_buffer[0]);
	// Compare calculated vs sent CRC values. ret = ESP_FAIL if not equal
	COMPARE_VAL(crc, read_buffer[2]);
	esp_err_t ret_temp = ret;
	if(ret_temp == ESP_FAIL)
	{
		ESP_LOGE(TAG, "%s, CRC check for temperature failed (INTEGER)", device_desc->name);
		ret = ESP_OK;
	}
	else 
	{
		// If ESP_OK calculate and put data into supplied pointer
		uint16_t temperature_data = ((uint16_t)read_buffer[0] << 8 | read_buffer[1]);
		*temperature = (-45 * SHT4X_INTEGER_PRECISION + ((uint64_t)175 * SHT4X_INTEGER_PRECISION * temperature_data) / 65535);
	}
	
	// CRC check humidity data
	crc = crc_check(&read_buffer[3]);
	// Compare calculated vs sent CRC values. ret = ESP_FAIL if not equal
	COMPARE_VAL(crc, read_buffer[5]);
	esp_err_t ret_humid = ret;
	if(ret_humid == ESP_FAIL)
	{
		ESP_LOGE(TAG, "%s, CRC check for humidity failed (INTEGER)", device_desc->name);
	}
	else 
	{
		// If ESP_OK calculate and put data into supplied pointer
		uint16_t humidity_data = ((uint16_t)read_buffer[3] << 8 | read_buffer[4]);
		*humidity = crop_humidity(humidity_data);
	}
	
	// Give back current device access semaphore and return
	xSemaphoreGive(device_desc->device_access_mutex);
	if(ret_temp == ESP_FAIL || ret_humid == ESP_FAIL)
		return ESP_FAIL;
	else
		return ESP_OK;	
	
	cleanup:
	xSemaphoreGive(device_desc->master_bus_mutex);
	xSemaphoreGive(device_desc->device_access_mutex);
	return ret;	
}

esp_err_t sht4x_read_float(sht4x_t *device_desc, float *temperature, float *humidity)
{
	// esp_err_t for ESP error handling macros
	esp_err_t ret = ESP_OK;
	
	// Take current device semaphore. Return ESP_ERR_TIMEOUT if timeout
	if(!xSemaphoreTake(device_desc->device_access_mutex, pdMS_TO_TICKS(SHT4X_DEVICE_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, DEVICE ACCESS SEMAPHORE TIMEOUT ON READ (FLOAT)", device_desc->name);
	}
	
	// Take port mutex which the current device is on. Send I2C read
	uint8_t read_buffer[DATA_READ_LENGTH] = {0};
	if(!xSemaphoreTake(device_desc->master_bus_mutex, pdMS_TO_TICKS(SHT4X_MASTER_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, MASTER BUS SEMAPHORE TIMEOUT ON READ (FLOAT)", device_desc->name);
	}
	ret = i2c_master_receive(device_desc->dev_handle, read_buffer, DATA_READ_LENGTH, SHT4X_TRANSACTION_TIMEOUT);
	ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "%s, I2C TEMP/HUMID READ FAILED", device_desc->name);
	xSemaphoreGive(device_desc->master_bus_mutex);
	
	// Check data validity via CRC
	// CRC check temperatues data 
	uint8_t crc = crc_check(&read_buffer[0]);
	// Compare calculated vs sent CRC values. ret = ESP_FAIL if not equal
	COMPARE_VAL(crc, read_buffer[2]);
	esp_err_t ret_temp = ret;
	if(ret_temp == ESP_FAIL)
	{
		ESP_LOGE(TAG, "%s, CRC check for temperature failed (FLOAT)", device_desc->name);
		ret = ESP_OK;
	}
	else 
	{
		// If ESP_OK calculate and put data into supplied pointer
		uint16_t temperature_data = ((uint16_t)read_buffer[0] << 8 | read_buffer[1]);
		*temperature = (-45 + (175.0f * temperature_data / 65535));
	}
	
	// CRC check humidity data
	crc = crc_check(&read_buffer[3]);
	// Compare calculated vs sent CRC values. ret = ESP_FAIL if not equal
	COMPARE_VAL(crc, read_buffer[5]);
	esp_err_t ret_humid = ret;
	if(ret_humid == ESP_FAIL)
	{
		ESP_LOGE(TAG, "%s, CRC check for humidity failed (FLOAT)", device_desc->name);
	}
	else 
	{
		// If ESP_OK calculate and put data into supplied pointer
		uint16_t humidity_data = ((uint16_t)read_buffer[3] << 8 | read_buffer[4]);
		*humidity = crop_humidity_float(humidity_data);
	}
	
	// Give back current device access semaphore and return based on if any crc checks failed
	xSemaphoreGive(device_desc->device_access_mutex);
	if(ret_temp == ESP_FAIL || ret_humid == ESP_FAIL)
		return ESP_FAIL;
	else
		return ESP_OK;	
	
	cleanup:
	xSemaphoreGive(device_desc->master_bus_mutex);
	xSemaphoreGive(device_desc->device_access_mutex);
	return ret;	
}

esp_err_t sht4x_read_serial(sht4x_t *device_desc, uint32_t *serial_number)
{
	// esp_err_t for ESP error handling macros
	esp_err_t ret = ESP_OK;
	
	// Take current device semaphore. Return ESP_ERR_TIMEOUT if timeout
	if(!xSemaphoreTake(device_desc->device_access_mutex, pdMS_TO_TICKS(SHT4X_DEVICE_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, DEVICE ACCESS SEMAPHORE TIMEOUT ON SERIAL NUMBER COMMAND", device_desc->name);
	}
	
	// Take port mutex which the current device is on. Send the measure command
	const uint8_t cmd = CMD_SERIAL;
	uint8_t read_buffer[DATA_READ_LENGTH] = {0};
	if(!xSemaphoreTake(device_desc->master_bus_mutex, pdMS_TO_TICKS(SHT4X_MASTER_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, MASTER BUS SEMAPHORE TIMEOUT ON SERIAL NUMBER COMMAND", device_desc->name);
	}
	ret = i2c_master_transmit_receive(device_desc->dev_handle, &cmd, CMD_LENGTH, read_buffer, DATA_READ_LENGTH, SHT4X_TRANSACTION_TIMEOUT);
	ESP_GOTO_ON_ERROR(ret, cleanup_w_master, TAG, "%s, I2C SERIAL NUMBER READ FAILED", device_desc->name);
	xSemaphoreGive(device_desc->master_bus_mutex);
	
	// Check data validity via CRC
	uint8_t crc = crc_check(&read_buffer[0]);
	// Compare calculated vs sent CRC values. ret = ESP_FAIL if not equal
	COMPARE_VAL(crc, read_buffer[2]);
	if(ret != ESP_OK)
	{
		ESP_LOGE(TAG, "%s, CRC check for serial number failed", device_desc->name);
		goto cleanup;
	}
	crc = crc_check(&read_buffer[3]);
	COMPARE_VAL(crc, read_buffer[5]);
	if(ret != ESP_OK)
	{
		ESP_LOGE(TAG, "%s, CRC check for serial number failed", device_desc->name);
		goto cleanup;
	}
	*serial_number = ((uint32_t)read_buffer[0] << 24 | (uint32_t)read_buffer[1] << 16 | (uint32_t)read_buffer[3] << 8 | (uint32_t)read_buffer[4]);
	
	cleanup:
	xSemaphoreGive(device_desc->device_access_mutex);
	return ret;

	cleanup_w_master:
	xSemaphoreGive(device_desc->master_bus_mutex);
	xSemaphoreGive(device_desc->device_access_mutex);
	return ret;	
}

esp_err_t sht4x_free_port(sht4x_i2c_master_bus_ctx_t *master_bus_ctx)
{
	// esp_err_t for ESP error handling macros
	esp_err_t ret = ESP_OK;
	
	// Make sure the port is not busy before deleting it
	if(!xSemaphoreTake(master_bus_ctx->master_bus_mutex, pdMS_TO_TICKS(SHT4X_MASTER_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "MASTER BUS SEMAPHORE TIMEOUT ON PORT DELETION");
	}
	// Delete port if successful delete the semaphore too
	ret = i2c_del_master_bus(master_bus_ctx->master_bus_handle);
	ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "PORT DELETION FAILED");
	
	vSemaphoreDelete(master_bus_ctx->master_bus_mutex);
	
	return ret;	
	
	cleanup:
	xSemaphoreGive(master_bus_ctx->master_bus_mutex);
	return ret;	
}

esp_err_t sht4x_free_device(sht4x_t *device_desc)
{
	// esp_err_t for ESP error handling macros
	esp_err_t ret = ESP_OK;
	
	// Take current device semaphore. Return ESP_ERR_TIMEOUT if timeout
	if(!xSemaphoreTake(device_desc->device_access_mutex, pdMS_TO_TICKS(SHT4X_DEVICE_MUTEX_TIMEOUT)))
	{
		ret = ESP_ERR_TIMEOUT;
		ESP_RETURN_ON_ERROR(ret, TAG, "%s, DEVICE ACCESS SEMAPHORE TIMEOUT ON DEVICE DELETION", device_desc->name);
	}
	// Delete device, its timer if successful delete the device access semaphore too
	ret = i2c_master_bus_rm_device(device_desc->dev_handle);
	ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "%s, DEVICE DELETION FAILED", device_desc->name);
	ret = esp_timer_delete(device_desc->timer);
	ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "%s, DEVICE TIMER DELETION FAILED", device_desc->name);
	
	vSemaphoreDelete(device_desc->device_access_mutex);
	
	return ret;
	
	cleanup:
	xSemaphoreGive(device_desc->device_access_mutex);
	return ret;	
}












































