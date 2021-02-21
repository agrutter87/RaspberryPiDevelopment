/* Need to use terminal output */
#include <iostream>
/* Need to use "pause()" function */
#include <unistd.h>
/* Need to use Wiring Pi functions */
#include <wiringPi.h>
/* Need to user Wiring Pi Serial functions */
#include <wiringSerial.h>
/* Need to use TFMini driver */
#include "TFMini.h"

/***********************************************
 * Prototype definitions
 ***********************************************/
void wiringpi_serial_init(int fd);
int wiringpi_serial_write(uint8_t val);
int wiringpi_serial_available(void);
int wiringpi_serial_read(void);
void wiringpi_serial_flush(void);
void wiringpi_delay(unsigned long ms);

/***********************************************
 * Program entry point
 ***********************************************/
int main(int argc, char *argv[])
{
    /* Holds file descriptor for serial port */
    int serial_fd = -1;

    /* Defines functions for TFMini driver to use to access Serial port */
    Stream wiringpi_stream =
    {
      .write = wiringpi_serial_write,
      .available = wiringpi_serial_available,
      .read = wiringpi_serial_read,
      .flush = wiringpi_serial_flush,
      .delay_ms = wiringpi_delay
    };

    /* Instantiate the TFMini class */
    TFMini tfmini;

    printf("Device Controller starting\n");

    /* Setup WiringPi */
    wiringPiSetup();

    /* Open WiringPi Serial */
    serial_fd = serialOpen("/dev/serial0", 115200);
    if(serial_fd == -1)
    {
        printf("Error opening serial port\n");
    }
    else
    {
        printf("Serial port opened successfully\n");

        wiringpi_serial_init(serial_fd);
		
        printf("Starting TFMini...");
        /* Initialize the TFMini with the WiringPi API */
        tfmini.begin(&wiringpi_stream, 1);

        /* Main loop */
        while(1)
        {
            // Take one TF Mini distance measurement
            uint16_t dist = tfmini.getDistance();
            uint16_t strength = tfmini.getRecentSignalStrength();

            // Display the measurement
            printf("%5d cm   sigstr: %5d\n", dist, strength);

            // Wait some short time before taking the next measurement
            delay(1000);
        }

        /* Close WiringPi Serial before exit */
        serialClose(serial_fd);
    }

    return 0;
}

static int wiringpi_serial_fd = -1;
void wiringpi_serial_init(int fd)
{
    wiringpi_serial_fd = fd;
}

int  wiringpi_serial_write(uint8_t val)
{
    serialPutchar(wiringpi_serial_fd, (unsigned char)val);

    return 1;
}

int wiringpi_serial_available(void)
{
    return serialDataAvail(wiringpi_serial_fd);
}

int wiringpi_serial_read(void)
{
    return serialGetchar(wiringpi_serial_fd);
}

void wiringpi_serial_flush(void)
{
    return serialFlush(wiringpi_serial_fd);
}

void wiringpi_delay(unsigned long ms)
{
    return delay((unsigned int)ms);
}
