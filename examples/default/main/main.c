#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/projdefs.h"
#include "sht4x.h"


#define I2C_PORT_1						1
#define SDA_PIN							4
#define SCL_PIN							5

#define INTEGER_FRACTION_SIZE			100


sht4x_i2c_master_bus_ctx_t master_bus = {0};
sht4x_t device = {0};

int32_t temperature, humidity;
float temperature_f, humidity_f;


// Calculates the whole and fractional part from the temperate and humidity huge integer values. Fraction is 2 decimal places. Change how many decimal places by INTEGER_FRACTION_SIZE
static inline void get_whole_and_fraction(int32_t full_value, int8_t *whole, int8_t *fraction)
{
	*whole = full_value / SHT4X_INTEGER_PRECISION;
	
	int32_t remainder = full_value % SHT4X_INTEGER_PRECISION;
	
	int32_t fraction_precision = SHT4X_INTEGER_PRECISION / INTEGER_FRACTION_SIZE;
	*fraction = remainder / fraction_precision;
	
	if (*fraction < 0)
    {
        *fraction = -(*fraction);
    }
}

// Initialize the master bus and the SHT4X device
static void initilization()
{
	i2c_master_bus_config_t master_bus_cfg = 
	{
		.i2c_port 		= I2C_PORT_1,
		.clk_source 	= I2C_CLK_SRC_XTAL,
		.scl_io_num		= SCL_PIN,
		.sda_io_num		= SDA_PIN
	};
		
	sht4x_i2c_master_bus_init(&master_bus, master_bus_cfg);
	sht4x_i2c_device_init(&master_bus, &device, "DEVICE_NUMBER_1", SHT4X_ADDR_1, FAST_MODE, false);
}


void app_main(void)
{
	int8_t whole_temp = 0, fraction_temp = 0;
	int8_t whole_humid = 0, fraction_humid = 0;
	
	while(1)
	{
		// Initialize the master bus and the SHT4X device
		initilization();
		for(int8_t i = 0; i < 4; i++)
		{
			// Set/Change device behavior
			device.heater = SHT4X_HEATER_OFF;				// OFF BY DEFAULT
			device.repeatability = SHT4X_REPEAT_HIGH;		// HIGH BY DEFAULT
			
			// Integers only
			sht4x_measure(&device);
			sht4x_read(&device, &temperature, &humidity);
			get_whole_and_fraction(temperature, &whole_temp, &fraction_temp);
			get_whole_and_fraction(humidity, &whole_humid, &fraction_humid);
			printf("(INTEGER) Temperature: %" PRId8 ".%" PRId8 "C Humidity: %" PRId8 ".%" PRId8 "%%", whole_temp, fraction_temp, whole_humid, fraction_humid);
			vTaskDelay(pdMS_TO_TICKS(1000));
			
			// Uses floats
			sht4x_measure(&device);
			sht4x_read_float(&device, &temperature_f, &humidity_f);
			printf("(FLOAT) Temperature: %fC, Humidity: %f%%", temperature_f, humidity_f);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		// Free all devices and port. Must free all devices associated with that port before deletion
		sht4x_free_device(&device);
		sht4x_free_port(&master_bus);
	}
}
