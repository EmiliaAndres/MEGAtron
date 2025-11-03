#include "space_protocol.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

// Define a maximum buffer size for CRC calculation
// This should be large enough to handle any frame size plus padding
#define CRC_BUFFER_SIZE 64


// Stolen from: https://stackoverflow.com/questions/54339800/how-to-modify-crc-32-to-crc-32-mpeg-2
uint32_t crc32b(unsigned char *message, size_t l)
{
   size_t i, j;
   unsigned int crc, msb;

   crc = 0xFFFFFFFF;
   for(i = 0; i < l; i++) {
      // xor next byte to upper bits of crc
      crc ^= (((unsigned int)message[i])<<24);
      for (j = 0; j < 8; j++) {    // Do eight times.
            msb = crc>>31;
            crc <<= 1;
            crc ^= (0 - msb) & 0x04C11DB7;
      }
   }
   return crc;         // don't complement crc on output
}

// CRC32 MPEG-2 pre-processing implementation to match Python implementation
uint32_t calculate_crc32_mpeg2(const uint8_t* data, size_t length) {
    // Pad data to multiple of 4 bytes
    size_t padded_length = length;
    if (length % 4 != 0) {
        padded_length += (4 - (length % 4));
    }
    
    // Static buffer for padded data
    uint8_t padded_data[CRC_BUFFER_SIZE];
    
    // Ensure we don't exceed the buffer size
    if (padded_length > CRC_BUFFER_SIZE) {
        return 0; // Error: buffer would overflow
    }
    
    // Zero out the buffer
    memset(padded_data, 0, padded_length);
    
    // Copy data to the static buffer
    memcpy(padded_data, data, length);
    
    // Convert to big-endian (swapping bytes) for each 32-bit word
    for (size_t i = 0; i < padded_length; i += 4) {
        uint8_t temp = padded_data[i];
        padded_data[i] = padded_data[i+3];
        padded_data[i+3] = temp;
        
        temp = padded_data[i+1];
        padded_data[i+1] = padded_data[i+2];
        padded_data[i+2] = temp;
    }
    
    // Calculate CRC using the crc32b function which matches the Python implementation
    return crc32b(padded_data, padded_length);
}



// Create a frame with specified values
Frame create_frame(uint8_t destination, uint8_t priority, uint8_t action, uint8_t source,
                  uint8_t device_type, uint8_t device_id, uint8_t data_type,
                  uint8_t operation, uint32_t payload) {
    Frame frame;
    frame.destination = destination;
    frame.priority = priority;
    frame.action = action;
    frame.source = source;
    frame.device_type = device_type;
    frame.device_id = device_id;
    frame.data_type = data_type;
    frame.operation = operation;
    frame.payload = payload;
    return frame;
}

// Pack a frame into bytes (with bit-level packing)
void pack_frame(const Frame* frame, uint8_t* buffer) {
    // Clear buffer
    memset(buffer, 0, FRAME_BYTE_LENGTH);
    
    // Header byte
    buffer[0] = HEADER_ID;
    
    // Pack frame fields using bit operations according to field definitions:
    // destination: 5 bits
    // priority: 2 bits
    // action: 4 bits
    // source: 5 bits
    // device_type: 6 bits
    // device_id: 6 bits
    // data_type: 4 bits
    // operation: 8 bits
    
    // First 16-bit value (little-endian): [destination(5)][priority(2)][action(4)][source(5)]
    uint16_t data_part_one = (frame->destination & 0x1F) |
                            ((frame->priority & 0x03) << 5) |
                            ((frame->action & 0x0F) << 7) |
                            ((frame->source & 0x1F) << 11);
    
    // Store data_part_one in little-endian format
    buffer[1] = data_part_one & 0xFF;
    buffer[2] = (data_part_one >> 8) & 0xFF;
    
    // Second 24-bit value (little-endian): [device_type(6)][device_id(6)][data_type(4)][operation(8)]
    uint32_t data_part_two = (frame->device_type & 0x3F) |
                            ((frame->device_id & 0x3F) << 6) |
                            ((frame->data_type & 0x0F) << 12) |
                            ((frame->operation & 0xFF) << 16);
    
    // Store data_part_two in little-endian format
    buffer[3] = data_part_two & 0xFF;
    buffer[4] = (data_part_two >> 8) & 0xFF;
    buffer[5] = (data_part_two >> 16) & 0xFF;
    
    // Pack payload (always 4 bytes) into bytes 6-9
    buffer[6] = (frame->payload >> 0) & 0xFF;
    buffer[7] = (frame->payload >> 8) & 0xFF;
    buffer[8] = (frame->payload >> 16) & 0xFF;
    buffer[9] = (frame->payload >> 24) & 0xFF;
}

// Encode a frame into a byte array, including bit reversal and CRC
void encode_frame(const Frame* frame, uint8_t* buffer) {
    // Pack the frame into the buffer
    pack_frame(frame, buffer);
    
    // Calculate CRC and append it to the buffer
    uint32_t crc = calculate_crc32_mpeg2(buffer, BODY_BYTE_LENGTH);
    
    // Copy CRC in little endian format
    buffer[BODY_BYTE_LENGTH + 0] = (crc >> 0) & 0xFF;
    buffer[BODY_BYTE_LENGTH + 1] = (crc >> 8) & 0xFF;
    buffer[BODY_BYTE_LENGTH + 2] = (crc >> 16) & 0xFF;
    buffer[BODY_BYTE_LENGTH + 3] = (crc >> 24) & 0xFF;
}

// Decode a byte array into a frame
int decode_frame(uint8_t* buffer, Frame* frame) {
    uint32_t received_crc, calculated_crc;
    
    // Extract CRC
    received_crc =((uint32_t)buffer[BODY_BYTE_LENGTH + 0]) |
                  ((uint32_t)buffer[BODY_BYTE_LENGTH + 1] << 8) |
                  ((uint32_t)buffer[BODY_BYTE_LENGTH + 2] << 16) |
                  ((uint32_t)buffer[BODY_BYTE_LENGTH + 3] << 24);
    
    // Calculate CRC
    calculated_crc = calculate_crc32_mpeg2(buffer, BODY_BYTE_LENGTH);
    
    // Check if CRC matches
    if (received_crc != calculated_crc) {
        return -1; // CRC mismatch
    }
    
    // Check header
    if (buffer[0] != HEADER_ID) {
        return -2; // Invalid header
    }
    
    // Unpack frame fields according to the Wireshark dissector format
    uint16_t data_part_one = ((uint16_t)buffer[1]) | ((uint16_t)buffer[2] << 8);
    uint32_t data_part_two = ((uint32_t)buffer[3]) | ((uint32_t)buffer[4] << 8) | ((uint32_t)buffer[5] << 16);
    
    frame->destination = data_part_one & 0x1F;
    frame->priority = (data_part_one >> 5) & 0x03;
    frame->action = (data_part_one >> 7) & 0x0F;
    frame->source = (data_part_one >> 11) & 0x1F;
    
    frame->device_type = data_part_two & 0x3F;
    frame->device_id = (data_part_two >> 6) & 0x3F;
    frame->data_type = (data_part_two >> 12) & 0x0F;
    frame->operation = (data_part_two >> 16) & 0xFF;
    
    // Unpack payload
    frame->payload = ((uint32_t)buffer[6]) |
                    ((uint32_t)buffer[7] << 8) |
                    ((uint32_t)buffer[8] << 16) |
                    ((uint32_t)buffer[9] << 24);
    
    return 0; // Success
}
