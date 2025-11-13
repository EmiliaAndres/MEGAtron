
#include <Arduino.h>
#include <space_protocol.h>
// include CanSatKit library used for pressure sensor
#include <CanSatKit.h>

#define NUM_FRAMES 2
#define TOTAL_PACKET_LENGTH (NUM_FRAMES * FRAME_BYTE_LENGTH) 

bool led_state = false;
const int led_pin = 13;
double T, P;

CanSatKit::Radio radio(CanSatKit::Pins::Radio::ChipSelect,
            CanSatKit::Pins::Radio::DIO0,
            433.0,
            CanSatKit::Bandwidth_125000_Hz,
            CanSatKit::SpreadingFactor_9,
            CanSatKit::CodingRate_4_8);

// BMP280 is a pressure sensor, create the sensor object
CanSatKit::BMP280 bmp;

uint8_t packet[NUM_FRAMES][FRAME_BYTE_LENGTH];

void get_bmp280_data(void)
{
  // start measurement, wait for result and save results in T and P variables 
  bmp.measureTemperatureAndPressure(T, P);
}
//send_frame
void pack_frame(float payloadValue,int sens_id,uint8_t* output_buffer)
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

    encode_frame(&frame, output_buffer);

}




void setup()
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

    get_bmp280_data();//get temperature and pressure

    pack_frame(T,1,packet[0]);
    pack_frame(P,2,packet[1]);
    radio.transmit((uint8_t*)packet, TOTAL_PACKET_LENGTH);

    //SerialUSB.write((uint8_t*)packet, TOTAL_PACKET_LENGTH);
    //SerialUSB.flush();
    delay(1000);

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