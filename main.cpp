#include "mbed.h"
#include <chrono>
#include <cstdint>
#include <cstring>

using namespace std::chrono;

constexpr int CONFIRM_TIME = 1000;
constexpr int BLINK_INTERVAL = 150;
constexpr int PAUSE_INTERVAL = 1000;
constexpr int ID_PAGE_ADDRESS = 0x0800F000;
constexpr int ANGLE_PAGE_ADDRESS = 0x0800F800;
constexpr int DEFAULT_CAN_ID = 0x450;
const uint16_t ADDR_DIAAGC = 0x3FFC; // 診断・AGCレジスタ
const uint16_t ADDR_ANGLE  = 0x3FFF; // 角度読み取りレジスタ

uint32_t stored_id = *((uint32_t*)ID_PAGE_ADDRESS);;
int64_t stored_angle = *((int64_t*)ANGLE_PAGE_ADDRESS);

DigitalOut can_led(PA_0);
DigitalOut id_indicator_led(PA_1);
DigitalOut angle_change_leds[2] = {DigitalOut(PA_3), DigitalOut(PA_4)};
InterruptIn id_set_button(PA_8, PullDown); // DigitalIn
SPI as5047p(PA_7, PA_6, PA_5);
DigitalOut css[2] = {DigitalOut(PB_0), DigitalOut (PB_1)};

BufferedSerial pc(USBTX, USBRX, 9600);
// BufferedSerial debug_uart(PA_9, PA_10, 9600);
CAN can(PA_11, PA_12, 1e6);
CANMessage send_msg, receive_msg;

Timer id_set_timer, can_check_timer;
Timeout blink_event, as5047_polling_event;

volatile int new_id_counter = 0;
int blink_count = 0;
uint16_t tderr_cnt = 0;
bool is_id_set_mode = false;
int16_t angle[2] = {0};
int32_t tmp = 0;

void id_set_isr();
void blink_handler();
int flash_write(uint32_t write_addr, uint32_t num);
uint16_t xfer(uint16_t addr, DigitalOut &cs) {
    // Readビット(bit14)を1にする
    uint16_t command = 0x4000 | addr;
    
    // 偶数パリティ計算 (bit15)
    // 0x3FFC なら 1, 0x3FFF なら 1 になる（簡易計算）
    command |= 0x8000; 

    cs = 0;
    uint16_t dummy = as5047p.write(command);
    cs = 1;
    wait_us(10);
    
    // 1つ前のコマンドの結果を取得
    cs = 0;
    uint16_t response = as5047p.write(command);
    cs = 1;
    return response;
}

int read_as5047p(){
        // 1. 角度データの取得とエラーフラグ(EF)の確認
        uint16_t angleData[2] = {0};
        bool errorFlag[2] = {false};
        uint16_t diaagc[2] = {0};
        bool vcc_err[2] = {false};  // VCC電圧異常
        bool magl[2] = {false};     // 磁力が弱すぎる（磁石なし）
        bool magh[2] = {false};     // 磁力が強すぎる
        bool cof[2] = {false};      // CORDIC計算オーバーフロー
        bool lf[2] = {false};       // オフセット補正完了フラグ(通常0)
        uint8_t agc[2] = {0};       // 自動利得制御の値(0-255)
        uint16_t angle[2] = {0};      

        for(int i = 0; i < 2; i++){
            angleData[i] = xfer(ADDR_ANGLE, css[i]);
            errorFlag[i] = (angleData[i] >> 14) & 0x01;
            
            // 2. 診断レジスタ(DIAAGC)の取得
            diaagc[i] = xfer(ADDR_DIAAGC[i], css[i]);

            // 各ビットの解析
           vcc_err[i] = (diaagc[i] >> 14) & 0x01; 
           magl[i] = (diaagc[i] >> 11) & 0x01; 
           magh[i] = (diaagc[i] >> 10) & 0x01; 
           cof[i] = (diaagc[i] >> 9)  & 0x01; 
           lf[i] = (diaagc[i] >> 8)  & 0x01; 
           agc[i] = diaagc[i] & 0xFF;         

            // エラー出力
            printf("[Status] ");
            if (!errorFlag[i] && !magl[i] && !magh[i] && !cof[i]) {
                printf("OK - ");
            } else {
                printf("ERROR DETECTED: ");
                if (errorFlag[i]) printf("| GLOBAL_ERR ");
                if (magl[i])      printf("| MAGNET_TOO_LOW (No Magnet?) ");
                if (magh[i])      printf("| MAGNET_TOO_HIGH ");
                if (cof[i])       printf("| CORDIC_OVERFLOW ");
                if (vcc_err[i])   printf("| VCC_VOLTAGE_ERROR ");
                printf("- ");
            }

            // 補足情報の出力
            angle[i] = angleData[i] & 0x3FFF;
            printf("Angle: %5u, AGC: %3u\n", angle[i], agc[i]);

            // 磁石の状態を分かりやすく表示
            if (magl[i]) {
                printf("  >> 磁石が離れすぎているか、装着されていません。\n");
            }
            if (magh[i]) {
                printf("  >> 磁石が近すぎます。チップとの距離を離してください。\n");
            }

            printf("--------------------------------------------------\n");
            ThisThread::sleep_for(1ms); 
            }
}

int main(){
    // printf("main function start\r\n");
    as5047p.format(16, 1);
    as5047p.frequency(10000000);
    css[0] = css[1] = 1;
    can.reset();
    id_set_timer.start();
    can_check_timer.start();
    id_set_button.rise(id_set_isr);
    
    blink_handler();

    angle_change_leds[0] = angle_change_leds[1] = false;

    while(true){
        angle[0] = angle[1] = 0;

        angle_change_leds[0] = (tmp != angle[0]);
        

        // if(is_id_set_mode) {
        //     id_indicator_led = id_set_button;
        //     if(id_set_timer.elapsed_time() > CONFIRM_TIME){
        //         if (flash_write(ID_PAGE_ADDRESS, new_id_counter) == 1) {
        //             NVIC_SystemReset();
        //         } else {
        //             is_id_set_mode = false;
        //             blink_handler();
        //         }
        //     }
        // }

        send_msg.id = DEFAULT_CAN_ID + stored_id;
        
        tderr_cnt = can.tderror();
        // tderr_cnt = 130; // test
        can_led = (tderr_cnt < 255);

        memcpy(send_msg.data, &angle, sizeof(angle));

        if(can.read(receive_msg)){
            if(receive_msg.id == (DEFAULT_CAN_ID + stored_id) && receive_msg.data[0] == 0xff) {
                NVIC_SystemReset();
                angle[0] = 0;
            }
        }
        can.write(send_msg);
        tmp = (int32_t)angle;

        // printf("%2d, %lld\r\n", stored_id, angle);
        // printf("\r\n%d, %d, ", stored_id, tderr_cnt);
        // for(int i = 0; i < 8; i++) printf("%03d ", send_msg.data[i]);
        
        ThisThread::sleep_for(10ms); // 1000Hz bad, 100Hz ok
    }
}

void id_set_isr(){
    if(!is_id_set_mode){
        // 最初のプレスでID設定モードを開始
        is_id_set_mode = true;
        new_id_counter = 1;
        id_set_timer.reset();
        blink_event.detach();
    } else {
        // ID設定モード中ならIDをカウントアップ
        new_id_counter++;
        id_set_timer.reset(); // タイマーリセットで猶予時間を延長
    }
}

void blink_handler(){
    if (stored_id == 0 || stored_id > 255) return;

    if(blink_count < (stored_id * 2)) {
        id_indicator_led = !id_indicator_led;
        blink_count++;
        blink_event.attach(blink_handler, static_cast<chrono::milliseconds>(BLINK_INTERVAL));
    } else {
        id_indicator_led = false;
        blink_count = 0;
        blink_event.attach(blink_handler, static_cast<chrono::milliseconds>(PAUSE_INTERVAL));
    }
}

int flash_write(uint32_t write_addr, uint32_t num){
    uint32_t page_error = 0;
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = write_addr;
    erase_init.NbPages = 1;
    if (HAL_OK != HAL_FLASHEx_Erase(&erase_init, &page_error)) return -1;
    if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_addr, num)) return -1;
    HAL_FLASH_Lock();
    return 1;
}

// #include "mbed.h"

// // BufferedSerial pc(PA_9, PA_10, 9600);
// InterruptIn id_set_button(PA_8, PullDown);
// DigitalOut id_indicator_led(PA_1);

// void handler();

// int main(){
//     id_set_button.rise(handler);
//     while(true){
//         ThisThread::sleep_for(10ms);
//     }
// }

// void handler(){
//     id_indicator_led = true;
// }

// #include "mbed.h"

// // UnbufferedSerial pc(USBTX, USBRX, 9600);
// CAN can(PA_11, PA_12, 1000000);
// CANMessage msg;

// int main(){
//     msg.data[0] = 1;
//     while(true){
//         can.write(msg);
//     }
// }


