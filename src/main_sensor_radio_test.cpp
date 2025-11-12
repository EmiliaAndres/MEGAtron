#include <Arduino.h>
#include <space_protocol.h>
// include CanSatKit library used for pressure sensor
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

// BMP280 is a pressure sensor, create the sensor object
CanSatKit::BMP280 bmp;

float get_bmp280_data(bool pressure)
{
  double T, P;
  // start measurement, wait for result and save results in T and P variables 
  bmp.measureTemperatureAndPressure(T, P);
  if(pressure)
  return P;
  else
  return T;
}

void send_frame(float payloadValue,int sens_id)
{

    uint32_t floatBits;
    memset(&floatBits, 0, sizeof(floatBits));
    memcpy(&floatBits, &payloadValue, sizeof(floatBits));

    Frame frame = create_frame(
        BOARD_GRAZYNA,
        PRIORITY_LOW,
        ACTION_FEED,
        BOARD_AGATKA,
        DEVICE_SENSOR,
        sens_id, // Sensor ID
        DATA_FLOAT,
        0x01, // Sensor read 0x01 operation
        floatBits);

    uint8_t packet[FRAME_BYTE_LENGTH];
    encode_frame(&frame, packet);

    //SerialUSB.write(packet, FRAME_BYTE_LENGTH);
    //SerialUSB.flush();
    radio.transmit(packet, FRAME_BYTE_LENGTH);
    #if BLINKING
    digitalWrite(led_pin, led_state);
    led_state = !led_state;//blink every 2 transmissions
    #endif
}


//void setup()
{
    SerialUSB.begin(115200);
    while(!bmp.begin()){};
    bmp.setOversampling(16);
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
    radio.begin();
    delay(50);//added delay not to overfill TX buffer
}

void loop()
{

    send_frame(get_bmp280_data(0),1);//send temp
    send_frame(get_bmp280_data(1),2);//send pressure
    static bool wait = false;

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