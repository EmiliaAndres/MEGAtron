#include <Arduino.h>
#include <space_protocol.h>
#include <CanSatKit.h>

#define BLINKING 0

bool led_state = false;
const int led_pin = 13;

CanSatKit::Radio radio(CanSatKit::Pins::Radio::ChipSelect,
            CanSatKit::Pins::Radio::DIO0,
            433.0,
            CanSatKit::Bandwidth_125000_Hz,
            CanSatKit::SpreadingFactor_9,
            CanSatKit::CodingRate_4_8);

//void setup()
{
    SerialUSB.begin(115200);
    pinMode(led_pin, OUTPUT);
    radio.begin();
}

void loop()
{

    static bool wait = false;
    static float payloadValue = 42.1f;

    uint32_t floatBits;
    memset(&floatBits, 0, sizeof(floatBits));
    memcpy(&floatBits, &payloadValue, sizeof(floatBits));

    Frame frame = create_frame(
        BOARD_GRAZYNA,
        PRIORITY_LOW,
        ACTION_FEED,
        BOARD_KROMEK,
        DEVICE_SENSOR,
        0x06, // Sensor ID
        DATA_FLOAT,
        0x01, // Sensor read 0x01 operation
        floatBits);

    uint8_t packet[FRAME_BYTE_LENGTH];
    encode_frame(&frame, packet);

    //SerialUSB.write(packet, FRAME_BYTE_LENGTH);
    //SerialUSB.flush();

    radio.transmit(packet, FRAME_BYTE_LENGTH);
    #if BLINKING == 1
    digitalWrite(led_pin, led_state);
    led_state = !led_state;//blink every 2 transmissions
    #endif

    
    // Increment payload for next transmission
    payloadValue = payloadValue + 10000;
    if (payloadValue > 100000) {
        payloadValue = -100000;
    }

    // Delay before sending the next frame
    if (wait)
    {
        delay(100); 
    }
    else
    {
        // First transmission happens immediately, then we wait
        wait = true;
        // Small delay after first transmission
        delay(1000);
    }

    // if data is available read all and do nothing with it
    if (SerialUSB.available())
    {
        while (SerialUSB.available())
        {
            SerialUSB.read();
        }
    }
}