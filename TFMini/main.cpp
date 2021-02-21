/* Need to use TFMini driver */
#include "TFMini.h"

/***********************************************
 * Program entry point
 ***********************************************/
int main(int argc, char *argv[])
{
	uint16_t dist = 0;
	uint16_t strength = 0;
	
    /* Instantiate the TFMini class */
    TFMini tfmini;

    printf("Device Controller starting\n");

	printf("Starting TFMini...");
	/* Initialize the TFMini with the WiringPi API */
	tfmini.begin(TFMINI_MODE_UART, 1);

	/* Main loop */
	while(1)
	{
		// Take one TF Mini distance measurement
		bool status = tfmini.read(&dist, &strength);
		if(status == true)
		{
			// Display the measurement
			printf("%5d cm   sigstr: %5d\n", dist, strength);
		}
		else
		{
			printf("Error taking measurement!\n");
		}

		// Wait some short time before taking the next measurement
		delay(1000);
	}

    return 0;
}