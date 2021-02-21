#include "TFMini.h"

TFMini::TFMini(void)
{
    // Empty constructor
}

TFMini::~TFMini(void)
{
	switch(mode)
	{
		case TFMINI_MODE_UART:
			/* Close WiringPi Serial before exit */
			serialClose(wiringpi_fd);
			break;
	    case TFMINI_MODE_I2C:
		    break;
		default:
		    break;
	}
}

/********************************************************************************************
 * TFMini::begin
 *******************************************************************************************/
bool TFMini::begin(unsigned int _mode, unsigned int _framerate_hz)
{
	mode = _mode;
	bool status = true;
	
    // Clear state
    distance = -1;
    strength = -1;
    state = READY;

    /* Setup WiringPi */
    wiringPiSetup();

    switch(mode)
	{
		case TFMINI_MODE_UART:
			/* Open WiringPi Serial */
			wiringpi_fd = serialOpen("/dev/serial0", 115200);
			if(wiringpi_fd == -1)
			{
				status = false;
				printf("Error opening serial port\n");
			}
		    break;
		
		case TFMINI_MODE_I2C:
		    break;
			
		default:
		    status = false;
	        break;
	}
	
	if(status == true)
	{
		printf("Serial port opened successfully\n");

		// Disable output on startup to make it easier to parse init command responses
		disableOutput();

		// Wait to allow for clearing of buffer
		delay(100);

		// Print the firmware version
		printFirmwareVersion();

		// Configure device
		setFrameRate(_framerate_hz);

		// Delay before beginning
		delay(1000);

		enableOutput();
	}

    return status;
}

/********************************************************************************************
 * TFMini::read
 *******************************************************************************************/
bool TFMini::read(uint16_t * _distance, uint16_t * _strength)
{
    int numMeasurementAttempts = 0;
    while (takeMeasurement() != 0)
    {
        numMeasurementAttempts += 1;
        if (numMeasurementAttempts > TFMINI_MAX_MEASUREMENT_ATTEMPTS)
        {
            printf("TF Mini error: too many measurement attempts\n");
            printf("Last error:");
            if (state == ERROR_SERIAL_NOHEADER)        printf("ERROR_SERIAL_NOHEADER\n");
            if (state == ERROR_SERIAL_BADCHECKSUM)     printf("ERROR_SERIAL_BADCHECKSUM\n");
            if (state == ERROR_SERIAL_TOOMANYTRIES)    printf("ERROR_SERIAL_TOOMANYTRIES\n");

            state = ERROR_SERIAL_TOOMANYTRIES;
            distance = -1;
            strength = -1;
            return -1;
        }
    }

    if (state == MEASUREMENT_OK)
    {
		*_distance = distance;
		*_strength = strength;
        return true;
    }
    else
    {
        return false;
    }
}

/********************************************************************************************
 * TFMini::disableOutput
 *******************************************************************************************/
void TFMini::disableOutput(void)
{
    serialFlush(wiringpi_fd);
    serialPutchar(wiringpi_fd, (unsigned char)0x5A);
    serialPutchar(wiringpi_fd, (unsigned char)0x05);
    serialPutchar(wiringpi_fd, (unsigned char)0x07);
    serialPutchar(wiringpi_fd, (unsigned char)0x00);
    serialPutchar(wiringpi_fd, (unsigned char)0x66);
}

/********************************************************************************************
 * TFMini::enableOutput
 *******************************************************************************************/
void TFMini::enableOutput(void)
{
    serialFlush(wiringpi_fd);
    serialPutchar(wiringpi_fd, (unsigned char)0x5A);
    serialPutchar(wiringpi_fd, (unsigned char)0x05);
    serialPutchar(wiringpi_fd, (unsigned char)0x07);
    serialPutchar(wiringpi_fd, (unsigned char)0x01);
    serialPutchar(wiringpi_fd, (unsigned char)0x67);
}

/********************************************************************************************
 * TFMini::triggerDetection
 *******************************************************************************************/
void TFMini::triggerDetection(void)
{
    serialFlush(wiringpi_fd);
    serialPutchar(wiringpi_fd, (unsigned char)0x5A);
    serialPutchar(wiringpi_fd, (unsigned char)0x04);
    serialPutchar(wiringpi_fd, (unsigned char)0x04);
    serialPutchar(wiringpi_fd, (unsigned char)0x62);
}

/********************************************************************************************
 * TFMini::setFrameRate
 *******************************************************************************************/
void TFMini::setFrameRate(unsigned int framerate_hz)
{
	switch(mode)
	{
		case TFMINI_MODE_UART:
			setFrameRateUART(framerate_hz);
			break;
		case TFMINI_MODE_I2C:
			setFrameRateI2C(framerate_hz);
			break;
		default:
			break;
	}
}

/********************************************************************************************
 * TFMini::setFrameRateUART
 *******************************************************************************************/
void TFMini::setFrameRateUART(unsigned int framerate_hz)
{
    uint8_t lsb = (uint8_t)framerate_hz;
    uint8_t msb = (uint8_t)(framerate_hz >> 8);
    uint8_t checksum = 0x5A + 0x06 + 0x03 + lsb + msb;

    uint8_t response[6];
    uint8_t resp_checksum = 0;

    serialFlush(wiringpi_fd);
    serialPutchar(wiringpi_fd, (unsigned char)0x5A);
    serialPutchar(wiringpi_fd, (unsigned char)0x06);
    serialPutchar(wiringpi_fd, (unsigned char)0x03);
    serialPutchar(wiringpi_fd, (unsigned char)(framerate_hz & 0xFF));
    serialPutchar(wiringpi_fd, (unsigned char)((framerate_hz >> 8) & 0xFF));
    serialPutchar(wiringpi_fd, (unsigned char)checksum);

    for (int i=0; i<6; i++)
    {
        // Read one character
        while (!serialDataAvail(wiringpi_fd))
        {
            // wait for a character to become available
        }
        response[i] = serialGetchar(wiringpi_fd);

        // Store running checksum
        if (i < 5)
        {
            resp_checksum += response[i];
        }
    }

    if (resp_checksum != response[5])
    {
        state = ERROR_SERIAL_BADCHECKSUM;
        distance = -1;
        strength = -1;
        if (TFMINI_DEBUGMODE == 1) printf("ERROR: bad checksum. calculated = 0x%x, received = 0x%x, difference = 0x%x\n", checksum, response[6], response[6] - checksum);
        return;
    }

    framerate = framerate_hz;
    printf("Framerate updated to %d Hz\n", framerate_hz);
}

/********************************************************************************************
 * TFMini::setFrameRateI2C
 *******************************************************************************************/
void TFMini::setFrameRateI2C(unsigned int framerate_hz)
{
	return;
}

/********************************************************************************************
 * TFMini::printFirmwareVersion
 *******************************************************************************************/
void TFMini::printFirmwareVersion(void)
{
	switch(mode)
	{
		case TFMINI_MODE_UART:
			printFirmwareVersionUART();
			break;
		case TFMINI_MODE_I2C:
			printFirmwareVersionI2C();
			break;
		default:
			break;
	}
}

/********************************************************************************************
 * TFMini::printFirmwareVersionUART
 *******************************************************************************************/
void TFMini::printFirmwareVersionUART(void)
{
    uint8_t response[7];
    uint8_t checksum = 0;

    serialFlush(wiringpi_fd);
    serialPutchar(wiringpi_fd, (unsigned char)0x5A);
    serialPutchar(wiringpi_fd, (unsigned char)0x04);
    serialPutchar(wiringpi_fd, (unsigned char)0x01);
    serialPutchar(wiringpi_fd, (unsigned char)0x5F);

    for (int i=0; i<7; i++)
    {
        // Read one character
        while (!serialDataAvail(wiringpi_fd))
        {
            // wait for a character to become available
        }
        response[i] = serialGetchar(wiringpi_fd);

        // Store running checksum
        if (i < 6)
        {
            checksum += response[i];
        }
    }

    if (checksum != response[6])
    {
        state = ERROR_SERIAL_BADCHECKSUM;
        distance = -1;
        strength = -1;
        if (TFMINI_DEBUGMODE == 1) printf("ERROR: bad checksum. calculated = 0x%x, received = 0x%x, difference = 0x%x\n", checksum, response[6], response[6] - checksum);
        return;
    }

    printf("Firmware version is V%d.%d.%d\n", response[5], response[4], response[3]);
}

/********************************************************************************************
 * TFMini::printFirmwareVersionI2C
 *******************************************************************************************/
void TFMini::printFirmwareVersionI2C(void)
{
	return;
}

/********************************************************************************************
 * TFMini::takeMeasurement
 *******************************************************************************************/
int TFMini::takeMeasurement(void)
{
	int ret = -1;
	switch(mode)
	{
		case TFMINI_MODE_UART:
			ret = takeMeasurementUART();
			break;
		case TFMINI_MODE_I2C:
			ret = takeMeasurementI2C();
			break;
		default:
			break;
	}
	
	return ret;
}

/********************************************************************************************
 * TFMini::takeMeasurementUART
 *******************************************************************************************/
int TFMini::takeMeasurementUART(void)
{
    int numCharsRead = 0;
    uint8_t lastChar = 0x00;

    if(framerate == 0)
    {
        // Trigger measurement
        triggerDetection();
    }

    // Step 1: Read the serial stream until we see the beginning of the TF Mini header, or we timeout reading too many characters.
    while(1)
    {
        if(serialDataAvail(wiringpi_fd))
        {
            uint8_t curChar = serialGetchar(wiringpi_fd);

            if ((lastChar == 0x59) && (curChar == 0x59))
            {
                // Break to begin frame
                break;
            }
            else
            {
                // We have not seen two 0x59's in a row -- store the current character and continue reading.
                lastChar = curChar;
                numCharsRead += 1;
            }
        }

        // Error detection: If we read more than X characters without finding a frame header, then it's likely there is an issue with
        // the Serial connection, and we should timeout and throw an error.
        if (numCharsRead > TFMINI_MAXBYTESBEFOREHEADER)
        {
            state = ERROR_SERIAL_NOHEADER;
            distance = -1;
            strength = -1;
            if (TFMINI_DEBUGMODE == 1) printf("ERROR: no header\n");
            return -1;
        }
    }

    // Step 2: Read one frame from the TFMini
    uint8_t frame[TFMINI_FRAME_SIZE];

    uint8_t checksum = 0x59 + 0x59;
    for (int i=0; i<TFMINI_FRAME_SIZE; i++)
    {
        // Read one character
        while (!serialDataAvail(wiringpi_fd))
        {
            // wait for a character to become available
        }
        frame[i] = serialGetchar(wiringpi_fd);

        // Store running checksum
        if (i < TFMINI_FRAME_SIZE-1)
        {
            checksum += frame[i];
        }
    }

    // Step 2A: Compare checksum
    // Last byte in the frame is an 8-bit checksum
    uint8_t checksumByte = frame[TFMINI_FRAME_SIZE-1];
    if (checksum != checksumByte)
    {
        state = ERROR_SERIAL_BADCHECKSUM;
        distance = -1;
        strength = -1;
        if (TFMINI_DEBUGMODE == 1) printf("ERROR: bad checksum. calculated = 0x%x, received = 0x%x, difference = 0x%x\n", checksum, checksumByte, checksumByte - checksum);
        return -1;
    }

    // Step 3: Interpret frame
    uint16_t dist = (frame[1] << 8) + frame[0];
    uint16_t st = (frame[3] << 8) + frame[2];
    uint8_t reserved = frame[4];
    uint8_t originalSignalQuality = frame[5];

    // Step 4: Store values
    distance = dist;
    strength = st;
    state = MEASUREMENT_OK;

    // Return success
    return 0;
}

/********************************************************************************************
 * TFMini::takeMeasurementI2C
 *******************************************************************************************/
int TFMini::takeMeasurementI2C(void)
{
	return 0;
}