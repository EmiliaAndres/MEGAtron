
#include <Arduino.h>
#include <space_protocol.h>
// include CanSatKit library used for pressure sensor
#include <CanSatKit.h>
#include <SPI.h>
#include <LoRa.h>

#define NUM_FRAMES 2
#define TOTAL_PACKET_LENGTH (NUM_FRAMES * FRAME_BYTE_LENGTH) 

bool led_state = false;
const int led_pin = 13;
double T, P;
unsigned int packet_id = 1;

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
    //LoRa configuration
    LoRa.setPins(10, -1, 12);
    LoRa.setSPIFrequency(4E6);
    while(!SerialUSB);//Wait for USB connection
    SerialUSB.println("LoRa Sender");
    delay(1000);
    if (!LoRa.begin(433E6)) {
        SerialUSB.println("Starting LoRa failed!");
        while (1);
    }
    LoRa.setSignalBandwidth(125E3);
    LoRa.setSpreadingFactor(9);
    LoRa.setCodingRate4(8);
    LoRa.setSyncWord(0x12);
    LoRa.setPreambleLength(8);
    //LoRa.explicitHeaderMode(); explicit header mode is turned on differently
    LoRa.enableCrc();
}

void loop()
{
    get_bmp280_data();//get temperature and pressure

    pack_frame(T,1,packet[0]);
    pack_frame(P,2,packet[1]);

    while (LoRa.beginPacket() == 0) {
        SerialUSB.println("waiting for radio ... ");
        delay(100);
    }
    delay(1000);
    SerialUSB.print("Packet has been sent, packet id :");
    SerialUSB.println(packet_id);
    //Asynchronus transmission
    LoRa.beginPacket();
    LoRa.write((uint8_t*)packet, TOTAL_PACKET_LENGTH);
    LoRa.endPacket(true); //LoRa.endPacket() - blocking mode, LoRa.endPacket(true) - non blocking mode
    packet_id++;

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