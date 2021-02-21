#include "TFMini.h"
#include <iostream>

TFMini::TFMini()
{
    // Empty constructor
}

/********************************************************************************************
 * TFMini::begin
 *******************************************************************************************/
bool TFMini::begin(Stream* _streamPtr, unsigned int framerate_hz)
{
    // Store reference to wiringPi file descriptor
    streamPtr = _streamPtr;

    // Clear state
    distance = -1;
    strength = -1;
    state = READY;

    // Disable output on startup to make it easier to parse init command responses
    disableOutput();

    // Wait to allow for clearing of buffer
    streamPtr->delay_ms(100);

    // Print the firmware version
    printFirmwareVersion();

    // Configure device
    setFrameRate(framerate_hz);

    // Delay before beginning
    streamPtr->delay_ms(1000);

    enableOutput();

    return true;
}

/********************************************************************************************
 * TFMini::getDistance
 *******************************************************************************************/
uint16_t TFMini::getDistance()
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
        return distance;
    }
    else
    {
        return -1;
    }
}

/********************************************************************************************
 * TFMini::getRecentSignalStrength
 *******************************************************************************************/
uint16_t TFMini::getRecentSignalStrength()
{
    return strength;
}

/********************************************************************************************
 * TFMini::disableOutput
 *******************************************************************************************/
void TFMini::disableOutput()
{
    streamPtr->flush();
    streamPtr->write((uint8_t)0x5A);
    streamPtr->write((uint8_t)0x05);
    streamPtr->write((uint8_t)0x07);
    streamPtr->write((uint8_t)0x00);
    streamPtr->write((uint8_t)0x66);
}

/********************************************************************************************
 * TFMini::enableOutput
 *******************************************************************************************/
void TFMini::enableOutput()
{
    streamPtr->flush();
    streamPtr->write((uint8_t)0x5A);
    streamPtr->write((uint8_t)0x05);
    streamPtr->write((uint8_t)0x07);
    streamPtr->write((uint8_t)0x01);
    streamPtr->write((uint8_t)0x67);
}

/********************************************************************************************
 * TFMini::triggerDetection
 *******************************************************************************************/
void TFMini::triggerDetection()
{
    streamPtr->flush();
    streamPtr->write((uint8_t)0x5A);
    streamPtr->write((uint8_t)0x04);
    streamPtr->write((uint8_t)0x04);
    streamPtr->write((uint8_t)0x62);
}

/********************************************************************************************
 * TFMini::setFrameRate
 *******************************************************************************************/
void TFMini::setFrameRate(unsigned int framerate_hz)
{
    uint8_t lsb = (uint8_t)framerate_hz;
    uint8_t msb = (uint8_t)(framerate_hz >> 8);
    uint8_t checksum = 0x5A + 0x06 + 0x03 + lsb + msb;

    uint8_t response[6];
    uint8_t resp_checksum = 0;

    streamPtr->flush();
    streamPtr->write((uint8_t)0x5A);
    streamPtr->write((uint8_t)0x06);
    streamPtr->write((uint8_t)0x03);
    streamPtr->write((uint8_t)(framerate_hz & 0xFF));
    streamPtr->write((uint8_t)((framerate_hz >> 8) & 0xFF));
    streamPtr->write((uint8_t)checksum);

    for (int i=0; i<6; i++)
    {
        // Read one character
        while (!streamPtr->available())
        {
            // wait for a character to become available
        }
        response[i] = streamPtr->read();

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
 * TFMini::printFirmwareVersion
 *******************************************************************************************/
void TFMini::printFirmwareVersion()
{
    uint8_t response[7];
    uint8_t checksum = 0;

    streamPtr->flush();
    streamPtr->write((uint8_t)0x5A);
    streamPtr->write((uint8_t)0x04);
    streamPtr->write((uint8_t)0x01);
    streamPtr->write((uint8_t)0x5F);

    for (int i=0; i<7; i++)
    {
        // Read one character
        while (!streamPtr->available())
        {
            // wait for a character to become available
        }
        response[i] = streamPtr->read();

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
 * TFMini::takeMeasurement
 *******************************************************************************************/
int TFMini::takeMeasurement()
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
        if(streamPtr->available())
        {
            uint8_t curChar = streamPtr->read();

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
        while (!streamPtr->available())
        {
            // wait for a character to become available
        }
        frame[i] = streamPtr->read();

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
