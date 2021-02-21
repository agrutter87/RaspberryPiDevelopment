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
    uint16_t getDistance();
    uint16_t getRecentSignalStrength();

  private:
    unsigned int mode;
    int wiringpi_fd;
    int state;
    uint16_t distance;
    uint16_t strength;
    unsigned int framerate;

    // Low-level communication
    void disableOutput();
    void enableOutput();
    void triggerDetection();
    void printFirmwareVersion();
    void setFrameRate(unsigned int framerate_hz);
    int takeMeasurement();
};
