/* Need to include for explicit size types */
#include <stdint.h>
/* Need to include IO functions for printf */
#include <cstdio>
/* Need to use Wiring Pi functions */
#include <wiringPi.h>
/* Need to user Wiring Pi Serial functions */
#include <wiringSerial.h>

// Defines
#define TFMINI_BAUDRATE                   (115200)
#define TFMINI_DEBUGMODE                  (1)

// The frame size is nominally 9 characters, but we don't include the first two 0x59's marking the start of the frame
#define TFMINI_FRAME_SIZE                 (7)

// Timeouts
#define TFMINI_MAXBYTESBEFOREHEADER       (30)
#define TFMINI_MAX_MEASUREMENT_ATTEMPTS   (10)

// States
#define READY                             (0)
#define ERROR_SERIAL_NOHEADER             (1)
#define ERROR_SERIAL_BADCHECKSUM          (2)
#define ERROR_SERIAL_TOOMANYTRIES         (3)
#define MEASUREMENT_OK                    (10)

// Modes
#define TFMINI_MODE_UART                  (0)
#define TFMINI_MODE_I2C                   (1)

//
// Driver Class Definition
//
class TFMini
{
  public:
    TFMini(void);
	~TFMini(void);

    // Configuration
    bool begin(unsigned int _mode, unsigned int _framerate_hz);

    // Data collection
    bool read(uint16_t * _distance, uint16_t * _strength);

  private:
    /* Mode of device, UART or I2C */
    unsigned int mode;
	/* File descriptor for either UART or I2C */
    int wiringpi_fd;
	/* Device state, used for error handling */
    int state;
	/* Last measured distance */
    uint16_t distance;
	/* Last measured strength */
    uint16_t strength;
	/* Device measurement framerate */
    unsigned int framerate;

    /* UART-only functions */
    void disableOutput(void);
    void enableOutput(void);
    void triggerDetection(void);
	
	/* Functions for printing firmware version */
    void printFirmwareVersion(void);
	void printFirmwareVersionUART(void);
	void printFirmwareVersionI2C(void);
	
	/* Functions for setting framerate */
    void setFrameRate(unsigned int framerate_hz);
	void setFrameRateUART(unsigned int framerate_hz);
	void setFrameRateI2C(unsigned int framerate_hz);
	
	/* Functions for taking measurements */
    int takeMeasurement(void);
	int takeMeasurementUART(void);
	int takeMeasurementI2C(void);
};
