#include <stdint.h>
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

/* Minimal struct definition for required Arduino functions.
 * An instance of this needs to be defined and passed to the
 * begin function to provide functions for the TFMini class to use */
struct Stream
{
    int    (* write)(uint8_t val);
    int    (* available)(void);
    int    (* read)(void);
    void   (* flush)(void);
    void   (* delay_ms)(unsigned long ms);
};

//
// Driver Class Definition
//
class TFMini
{
  public:
    TFMini(void);

    // Configuration
    bool begin(Stream* _streamPtr, unsigned int framerate_hz);

    // Data collection
    uint16_t getDistance();
    uint16_t getRecentSignalStrength();

  private:
    Stream* streamPtr;
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
