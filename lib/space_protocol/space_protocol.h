#ifndef SPACE_PROTOCOL_H
#define SPACE_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif



#endif // SPACE_PROTOCOL_H

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdlib.h>

// Constants from the Python implementation
#define HEADER_ID 0x5
#define HEADER_BYTE_LENGTH 1
#define PAYLOAD_BYTE_LENGTH 4
#define CRC_BYTE_LENGTH 4
#define BODY_BYTE_LENGTH (HEADER_BYTE_LENGTH + 5 + PAYLOAD_BYTE_LENGTH)
#define FRAME_BYTE_LENGTH (BODY_BYTE_LENGTH + CRC_BYTE_LENGTH)

// Board IDs
typedef enum {
    BOARD_GRAZYNA = 0x01,
    BOARD_STASZEK = 0x02,
    BOARD_RADEK = 0x03,
    BOARD_CZAPLA = 0x04,
    BOARD_PAUEK = 0x05,
    BOARD_KROMEK = 0x06,
    BOARD_ANTEK = 0x07,
    BOARD_OLA = 0x08,
    BOARD_LAST_BOARD = 0x09,
    BOARD_AGATKA = 0x0A,
    BOARD_BARTEK = 0x0B,
    BOARD_PIETEK = 0x0C,
    BOARD_PROXY = 0x1E,
    BOARD_BROADCAST = 0x1F
} BoardID;

// Device IDs
typedef enum {
    DEVICE_SERVO = 0x00,
    DEVICE_RELAY = 0x01,
    DEVICE_SENSOR = 0x02,
    DEVICE_PISTON = 0x02,
    DEVICE_SUPPLY = 0x03,
    DEVICE_MEMORY = 0x04,
    DEVICE_IGNITER = 0x05,
    DEVICE_FLASH = 0x06,
    DEVICE_MPU9250 = 0x08,
    DEVICE_DYNAMIXEL = 0x09,
    DEVICE_SCHEDULER = 0x0A,
    DEVICE_RECOVERY = 0x0B,
    DEVICE_PARACHUTE = 0x0C,
    DEVICE_RESET = 0x0D,
    DEVICE_KEEPALIVE = 0x0E,
    DEVICE_HEATINGLAMP = 0x01
} DeviceID;

// Action IDs
typedef enum {
    ACTION_FEED = 0x00,
    ACTION_SERVICE = 0x01,
    ACTION_ACK = 0x02,
    ACTION_NACK = 0x03,
    ACTION_HEARTBEAT = 0x04,
    ACTION_REQUEST = 0x05,
    ACTION_RESPONSE = 0x06,
    ACTION_SCHEDULE = 0x07,
    ACTION_SACK = 0x08,
    ACTION_SNACK = 0x09
} ActionID;

// Data Type IDs
typedef enum {
    DATA_NO_DATA = 0x00,
    DATA_UINT32 = 0x01,
    DATA_UINT16 = 0x02,
    DATA_UINT8 = 0x03,
    DATA_INT32 = 0x04,
    DATA_INT16 = 0x05,
    DATA_INT8 = 0x06,
    DATA_FLOAT = 0x07,
    DATA_INT16X2 = 0x08,
    DATA_UINT16INT16 = 0x09
} DataTypeID;

// Priority IDs
typedef enum {
    PRIORITY_HIGH = 0x00,
    PRIORITY_LOW = 0x01
} PriorityID;

// Frame structure - maps to Python Frame class
typedef struct {
    uint8_t destination;  // 5 bits
    uint8_t priority;     // 2 bits
    uint8_t action;       // 4 bits
    uint8_t source;       // 5 bits
    uint8_t device_type;  // 6 bits
    uint8_t device_id;    // 6 bits
    uint8_t data_type;    // 4 bits
    uint8_t operation;    // 8 bits
    uint32_t payload;     // 32 bits (4 bytes)
} Frame;

// Function prototypes
/**
 * Calculate CRC32 MPEG-2 for the given data
 * @param data Pointer to the data buffer
 * @param length Length of the data in bytes
 * @return CRC32 MPEG-2 value
 */
uint32_t calculate_crc32_mpeg2(const uint8_t* data, size_t length);

/**
 * Create a new frame with the specified values
 * @param destination Target board ID
 * @param priority Message priority
 * @param action Action type
 * @param source Source board ID
 * @param device_type Target device type
 * @param device_id Target device ID
 * @param data_type Type of data in the payload
 * @param operation Operation to perform
 * @param payload Data payload (up to 32 bits)
 * @return Configured Frame structure
 */
Frame create_frame(uint8_t destination, uint8_t priority, uint8_t action, uint8_t source,
                  uint8_t device_type, uint8_t device_id, uint8_t data_type,
                  uint8_t operation, uint32_t payload);

/**
 * Pack a frame into a byte buffer
 * @param frame Pointer to the frame to pack
 * @param buffer Buffer to store the packed frame (must be at least FRAME_BYTE_LENGTH bytes)
 */
void pack_frame(const Frame* frame, uint8_t* buffer);

/**
 * Encode a frame into a byte array, including bit reversal and CRC
 * @param frame Pointer to the frame to encode
 * @param buffer Buffer to store the encoded frame (must be at least FRAME_BYTE_LENGTH bytes)
 */
void encode_frame(const Frame* frame, uint8_t* buffer);

/**
 * Decode a byte array into a frame
 * @param buffer Buffer containing the encoded frame (must be at least FRAME_BYTE_LENGTH bytes)
 * @param frame Pointer to store the decoded frame
 * @return 0 on success, -1 on CRC mismatch, -2 on invalid header
 */
int decode_frame(uint8_t* buffer, Frame* frame);


#endif /* PROTOCOL_H */

#ifdef __cplusplus
}
#endif