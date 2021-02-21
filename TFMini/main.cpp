/* Need to use TFMini driver */
#include "TFMini.h"

/***********************************************
 * Program entry point
 ***********************************************/
int main(int argc, char *argv[])
{
	uint16_t uart_dist = 0;
	uint16_t uart_strength = 0;
	uint16_t i2c_dist = 0;
	uint16_t i2c_strength = 0;
	
	bool status;
	
    /* Instantiate the UART TFMini class */
    TFMini tfmini_uart;
	/* Instantiate the I2C TFMini class */
	TFMini tfmini_i2c;

    printf("Device Controller starting\n");

	printf("Starting UART TFMini...");
	/* Initialize the TFMini with the WiringPi API */
	tfmini_uart.begin(TFMINI_MODE_UART, 1);
	tfmini_i2c.begin(TFMINI_MODE_I2C, 1);

	/* Main loop */
	while(1)
	{
		// Take one TF Mini distance measurement
		status = tfmini_uart.getReadings(&uart_dist, &uart_strength);
		if(status == true)
		{
			// Display the measurement
			printf("UART: %5d cm   sigstr: %5d\n", uart_dist, uart_strength);
		}
		else
		{
			printf("UART: Error taking measurement!\n");
		}

		// Take one TF Mini distance measurement
		status = tfmini_i2c.getReadings(&i2c_dist, &i2c_strength);
		if(status == true)
		{
			// Display the measurement
			printf("I2C:  %5d cm   sigstr: %5d\n", i2c_dist, i2c_strength);
		}
		else
		{
			printf("I2C:  Error taking measurement!\n");
		}

		// Wait some short time before taking the next measurement
		delay(1000);
	}

    return 0;
}