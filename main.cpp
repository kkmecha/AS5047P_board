#include "mbed.h"
#include <cstdint>
#include <cstring>

BufferedSerial pc(USBTX, USBRX, 115200);
SPI spi(PA_7, PA_6, PA_5); 
DigitalOut cs(PA_4);
CAN can(PA_11, PA_12, 1000000);
CANMessage msg, data;
DigitalIn light_sensor(PB_6, PullUp);
DigitalIn light(PB_7, PullUp);

#define magid 0x01

const uint16_t ADDR_DIAAGC = 0x3FFC; 
const uint16_t ADDR_ANGLE  = 0x3FFF; 

// === 累積角計算用の変数（エンコーダ側で高速処理） ===
bool initialized_angle = false;
float prev_raw_deg = 0.0f;
float accumulated_deg = 0.0f;

uint16_t xfer(uint16_t addr) {
    uint16_t command = 0x4000 | addr;
    command |= 0x8000; 

    cs = 0;
    uint16_t dummy = spi.write(command);
    cs = 1;
    wait_us(10);
    
    cs = 0;
    uint16_t response = spi.write(command);
    cs = 1;
    return response;
}

int main() {
    spi.format(16, 1);
    spi.frequency(10000000);
    cs = 1;

    printf("--- AS5047P Full Diagnostics System Start ---\n");
    
    while (true) {
        data.id = 0x402;
        data.len = 8;

        // 1. 角度データの取得
        uint16_t angleData = xfer(ADDR_ANGLE);
        uint16_t raw_angle = angleData & 0x3FFF; // 14bitマスク

        // 0〜16383 を 0.0〜360.0度に変換
        float current_deg = (raw_angle / 16384.0f) * 360.0f;

        if (!initialized_angle) {
            prev_raw_deg = current_deg;
            accumulated_deg = 0.0f; 
            initialized_angle = true;
        } else {
            float delta = current_deg - prev_raw_deg;
            
            // 境界またぎ補正
            if (delta > 180.0f)       delta -= 360.0f;
            else if (delta < -180.0f) delta += 360.0f;
            
            // ★エンコーダ自体の正負をここで反転させたい場合は `-= delta` にする
            accumulated_deg += delta; 
            prev_raw_deg = current_deg;
        }

        // 制御に使う型に合わせて int32_t にキャスト
        int32_t send_angle = (int32_t)accumulated_deg;

        // 2. 診断レジスタの取得
        uint16_t diaagc = xfer(ADDR_DIAAGC);
        uint8_t agc  = diaagc & 0xFF;

        // シリアル出力（デバッグ用）
        printf("Angle(Deg): %ld, AGC: %3u, tderr: %d, rderr: %d\n", send_angle, agc, can.tderror(), can.rderror());

        // === CANデータ詰め込み ===
        // 0〜3バイト：32bitの累積角（度）
        std::memcpy(&data.data[0], &send_angle, sizeof(int32_t));
        
        // 4〜5バイト：予備（元の raw_angle を一応残しておく場合）
        std::memcpy(&data.data[4], &raw_angle, sizeof(uint16_t));

        // 6〜7バイト：センサー類
        data.data[6] = light_sensor.read();
        data.data[7] = light.read();

        can.write(data);
        
        // ★ここで超高速（1ms未満）で回すことで、エイリアシングを完全に防ぐ
        thread_sleep_for(1); 
    }
}