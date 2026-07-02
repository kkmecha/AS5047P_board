#include "mbed.h"
#include <cstdint>
#include <cstring>

BufferedSerial pc(USBTX, USBRX, 921600);
SPI spi(PA_7, PA_6, PA_5); 
DigitalOut cs[2] = {DigitalOut(PA_4), DigitalOut(PA_2)};
CAN can(PA_11, PA_12, 1000000);
CANMessage msg, data;
DigitalIn light_sensor(PF_1, PullUp);
DigitalIn light(PF_0, PullUp);

#define magid 0x01

const uint16_t ADDR_DIAAGC = 0x3FFC; 
const uint16_t ADDR_ANGLE  = 0x3FFF; 

bool initialized_angle[2] = {false, false};
float prev_raw_deg[2] = {0.0f, 0.0f};
float accumulated_deg[2] = {0.0f, 0.0f};

uint16_t xfer(uint16_t addr, int ch) {
    uint16_t command = 0x4000 | addr;
    command |= 0x8000; 

    cs[ch] = 0;
    uint16_t dummy = spi.write(command);
    cs[ch] = 1;
    wait_us(10);
    
    cs[ch] = 0;
    uint16_t response = spi.write(command);
    cs[ch] = 1;
    return response;
}

int main() {
    spi.format(16, 1);
    spi.frequency(10000000);
    
    cs[0] = 1;
    cs[1] = 1;

    
    while (true) {
        CANMessage rcv_msg;
        if (can.read(rcv_msg)) {  
            if(rcv_msg.id == 0x400){
                int cnt = 0;
                for(int i = 0; i < 8; i++) {
                    if(rcv_msg.data[i] == 0xff) cnt++;  
                }
                if(cnt == 8) NVIC_SystemReset();
            }
        }

        for (int i = 0; i < 2; i++) {
            CANMessage data;
            data.id = 0x402 + i;
            data.len = 8;

            uint16_t angleData = xfer(ADDR_ANGLE, i);
            uint16_t raw_angle = angleData & 0x3FFF; 

            float current_deg = (raw_angle / 16384.0f) * 360.0f;

            if (!initialized_angle[i]) {
                prev_raw_deg[i] = current_deg;
                accumulated_deg[i] = 0.0f; 
                initialized_angle[i] = true;
            } else {
                float delta = current_deg - prev_raw_deg[i];
                
                if (delta > 180.0f)       delta -= 360.0f;
                else if (delta < -180.0f) delta += 360.0f;
                
                accumulated_deg[i] += delta; 
                prev_raw_deg[i] = current_deg;
            }

            int32_t send_angle = (int32_t)(accumulated_deg[i] * 100.0f);

            // uint16_t diaagc = xfer(ADDR_DIAAGC, i);
            // uint8_t agc  = diaagc & 0xFF;
            std::memcpy(&data.data[0], &send_angle, sizeof(int32_t));
            std::memcpy(&data.data[4], &raw_angle, sizeof(uint16_t));

            data.data[6] = light_sensor.read();
            data.data[7] = light.read();

            can.write(data);

            printf("%d\r\n", send_angle);
        }
        
        thread_sleep_for(1); 
    }
}