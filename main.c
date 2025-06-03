// tiger16 受信機メインdsPIC
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "soft_i2c.h"
#include "lcd_i2c.h"
#include "twe_lite.h"
#include "crc16.h"


#define USE_LCD 0 /* 液晶ディスプレイを使う:1 使わない:0 */
#define MIN_TRG_OFF 3 /* トリガーオフとみなす連続オフ回数 */
#define TIM_MAG 150 /* 電動マガジン通電時間 ms */

#define MAX_CANNON 520000    /* 大きくすると俯角が大きく取れる */
#define MIN_CANNON (-640000) /* 小さくすると仰角が大きく取れる */


typedef union tagHL16 {
    signed short SHL;
    uint16_t HL;
    struct {
        uint8_t L;
        uint8_t H;
    };
    struct {
        unsigned :8;
        unsigned :7;
        unsigned T:1;
    };
} HL16;


typedef union tagHL32 {
    unsigned long HL;
    struct {
        uint8_t L;
        uint16_t M;
        uint8_t H;
    };
} HL32;


uint16_t table_pwm[] = {
	10,
	11,
	12,
	13,
	15,
	16,
	18,
	19,
	21,
	24,
	26,
	29,
	31,
	35,
	38,
	42,
	46,
	51,
	56,
	61,
	67,
	74,
	81,
	90,
	98,
	108,
	119,
	131,
	144,
	159,
	174,
	192,
	211,
	267,
	323,
	379,
	435,
	491,
	546,
	602,
	658,
	714,
	770,
	826,
	882,
	938,
	994,
	1050,
	1106,
	1161,
	1217,
	1273,
	1329,
	1385,
	1441,
	1497,
	1553,
	1609,
	1665,
	1720,
	1776,
	1832,
	1888,
	1944,
	2000,
	2312,
	2625,
	2938,
	3250,
	3562,
	3875,
	4188,
	4500,
	4812,
	5125,
	5438,
	5750,
	6062,
	6375,
	6688,
	7000,
	7312,
	7625,
	7938,
	8250,
	8562,
	8875,
	9188,
	9500,
	9812,
	10125,
	10438,
	10750,
	11062,
	11375,
	11688,
	12000,
	12312,
	12625,
	12938,
	13250,
	13562,
	13875,
	14188,
	14500,
	14812,
	15125,
	15438,
	15750,
	16062,
	16375,
	16688,
	17000,
	17312,
	17625,
	17938,
	18250,
	18562,
	18875,
	19188,
	19500,
	19812,
	20125,
	20438,
	20750,
	21062,
	21375,
	21688,
	21688
};            


// AD変換値 DMAセットされる
//uint16_t temp1 = 0;
//uint16_t temp2 = 0;
uint16_t num_batt, battery = 0;
unsigned long sum_batt = 0;

#define TIMEOUT 400 /* 受信タイムアウト */
uint8_t sid[] = {0x82, 0x02, 0x2e, 0x90}; // 送信機の TWE LITE シリアル番号
#define RSV_BYTES 8 /* 電波受信すべきデーターのバイト数 送信機 send_main に応じた値でなければならない */ 
#define SPI_BYTES 8 /* SPI送受信するデーターのバイト数 */
#define MAX_CNT_ERR 5 /* 連続エラーがこれだけ続くと強制停止 */
//#define LOADING_COUNT 20 /* 装填動作待ち時間（片道・30ミリ秒単位）*/
//#define CNT_TRIG 10 /* トリガーON持続時間（30ミリ秒単位）*/
//#define CNT_AFTER 40 /* 射撃後の充電開始待ち時間（30ミリ秒単位）*/

uint8_t data[SPI_BYTES]; // SPI受信格納先
    // data[0] 温度
uint8_t send[SPI_BYTES]; // SPI送信格納先


uint16_t cnt_err = 0; // ERROR 連続回数
//HL16 data_ok; // 正常に受信できた最後のデーター 0:強制停止

// サーボ値
signed long cann1 = 0; // 仰角
uint8_t err1 = 0; // エラー
uint8_t in_fire = 0; // 発射通電中なら1
uint16_t fired = 0; // 射撃弾数
uint16_t cnt_mag = 0; // マガジン通電残時間
uint8_t cnt_laser = 0; // レーザー点滅カウンター

uint8_t rsvt[32]; // 受信バッファー
#define RSVA_BYTES 24 
uint8_t rsv[RSVA_BYTES]; // 正常に受信できたデーターの転送先
char buf[32];


 // TMR2 割り込みごとにカウントダウンする。１カウント30ミリ秒



// SPI送信
void spi_send(void) {
    uint8_t idx, b, d, d2, m, *dp = data;
    // パケット先頭 STRB=1 で相手に伝える
    // クロックを1にするまで15μ秒以上空けるのを仕様とする
    SPI_STRB_SetHigh();
    __delay_us(14);

    SPI_STRB_SetLow();
    for (idx=0; idx<SPI_BYTES; idx++) {
        d = 0;
        d2 = send[idx];
        m = 0x80;
        for (b=0; b<8; b++) {
            SPI_CLOCK_SetHigh();
            if (d2 & m) {
                SPI_DATA_SetHigh();
            }
            else {
                SPI_DATA_SetLow();
            }
            __delay_us(3);
            m >>= 1;
            d <<= 1;
            SPI_CLOCK_SetLow();
            d |= SPI_IN_GetValue();
            __delay_us(3);
        }        
        (*(dp++)) = d;
    }
    SPI_DATA_SetLow();
}


// 受信データー確認
// 受信あれば1 なければ0 を返す
char check_rsv(void) {
    uint8_t i, n = get_rsv_size();
    if (n <= 15) {
        return 0; // 受信データーが少な過ぎる
    }
    // 送信機のシリアルIDを確認
    for (i=0; i<4; i++) {
        if (rsvt[i+3] != sid[i]) {
            return 0; // 送信機のシリアルIDと違う
        }
    }
    if (rsvt[13] != RSV_BYTES) {
        return 0; // データー長が想定と違う
    }

    for (i=0; i<RSVA_BYTES; i++) {
        rsv[i] = rsvt[i];
    }
    return 1;
}


// 30ms ごと
void int_timer1(void) {
    cnt_laser ++;
    if (cnt_laser & 3) {
        LASER_SetHigh();
    }
    else {
        if (rsv[15] & 16) {
            LASER_SetHigh();
        }
        else {
            LASER_SetLow();
        }
    }
}



// 射撃通電から時間が経ちすぎていれば通電停止＆エラー
void int_timer2(void) {
    FIRE_SetLow(); // 通電OFF
    TMR2_Stop();
    in_fire = 0;
    err1 = 1;
    cnt_mag = 0;
}




// セミオートスイッチ状態変化
void int_semi(void) {
    TMR2_Stop();
    if (in_fire == 0) return;
    if (SEMIAUTO_GetValue()) { // ON→OFFになった
        // サイクル終了
        FIRE_SetLow(); // 通電OFF
        in_fire = 0;
        fired ++;
        cnt_mag = TIM_MAG;
    }
}
 

int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    UART1_SetRxInterruptHandler(TWE_rsv_int);
    set_rsv_buf(rsvt, 32);
    TMR1_SetInterruptHandler(int_timer1);
    TMR2_SetInterruptHandler(int_timer2);
    TMR2_Stop();
    CN_SetInterruptHandler(int_semi);
    DMA_ChannelEnable(DMA_CHANNEL_1);
    DMA_PeripheralAddressSet(DMA_CHANNEL_1, (volatile unsigned int) &ADC1BUF0);
    DMA_StartAddressASet(DMA_CHANNEL_1, (uint16_t)(&battery));        

#if (USE_LCD)
    __delay_ms(100);    
    WATCHDOG_TimerClear();
    LCD_i2c_init(8);
#endif

    PDC3 = 6542;
    uint8_t cnt = 200;
    while (SEMIAUTO_GetValue() == 0) { // セミオートスイッチが最初から入っている
        // 最大２００ミリ秒まで通電しセミオートスイッチOFF状態にする
        FIRE_SetHigh(); // 通電ON
        __delay_ms(1);
        cnt --;
        if (cnt == 0) {
            err1 = 1;
            break;
        }
    }
    FIRE_SetLow(); // 通電OFF
    in_fire = 0;
    fired = 0;
    
    uint8_t id = 0; // 応答ID
    uint8_t WiFi = 0; // 受信感度
    uint8_t i;
    uint16_t t = 0;

    for (i=0; i<RSVA_BYTES; i++) {
        rsv[i] = 0;
    }
    for (i=0; i<SPI_BYTES; i++) {
        data[i] = send[i] = 0;
    }

    uint8_t trigger = 0; // トリガー押されたら1（ナマ値）
    uint8_t trigger1 = 0; // トリガー押されたら1（有効値）
    
    
    // 直前の動作モード
    uint8_t mode0 = 0;
  
    // 現在の動作モード
    uint8_t mode = 1;
    // 1: 受信待ち
    // 2: 受信成功時
    // 3: 受信失敗時
    uint16_t tim_rsv = 0; // 受信待ちタイムアウトカウンター
    
    uint8_t cnt_trig_off = 0; // トリガー押されていない連続回数
    
    while (1)
    {
        WATCHDOG_TimerClear();
        trigger1 = 0;
    
        if (mode == 1) { // 受信待ち
            if (mode0 != mode) { // モードが変わった直後
                tim_rsv = 0; // 受信待ちタイムアウトカウンター
                // バッテリー電圧集計開始
                num_batt = 1;
                sum_batt = (unsigned long)battery;
            }
            else {
                tim_rsv ++;
                // バッテリー電圧集計
                num_batt ++;
                sum_batt += (unsigned long)battery;
            }
            __delay_ms(1);
            if (check_rsv()) {
                mode0 = mode;
                mode = 2; // 受信成功
                clear_rsv_size();
            }
            else if (tim_rsv >= TIMEOUT) {
                mode0 = mode;
                mode = 3; // 受信失敗
                clear_rsv_size();
            }
        }
        else { // 受信終了
            if (mode == 2) { // 受信成功
                // rsv[2] 応答ID
                id = rsv[2];
                // rsv[11] 受信強度
                WiFi = rsv[11];
            
                // rsv[14]
                // bit0  TRR-U
                // bit1  TRR-D
                // bit2  TRR-L
                // bit3  TRR-R
                // bit4  B（LCD周囲のボタン）
                // bit5  A（LCD周囲のボタン）
                // bit6  Down（LCD周囲のボタン）
                // bit7  Up（LCD周囲のボタン）

                // rsv[15]
                // bit0  TRL-U
                // bit1  TRL-D
                // bit2  TRL-L
                // bit3  TRL-R
                // bit4  左トグルスイッチが↓なら1
                // bit5  トリガーが押されたら1
                // bit6  左トリガーが押されたら1
                // bit7  トリガー同時押しで1

                // rsv[16] と [17] は使われていない
            
                // rsv[18] スロットル　右十字の上下　上で＋
                // rsv[19] エルロン　右十字の左右　右で＋
                // rsv[20] 旋回　左十字の左右　左で＋
                // rsv[21] 上下　左十字の上下　上で＋

                // 送信機の入力
                send[0] = rsv[14];
                send[1] = rsv[15];
                send[2] = rsv[18];
                send[3] = rsv[19];
                send[4] = rsv[20];
                send[5] = rsv[21];

                // CRC16
                HL16 crc;
                uint8_t const *p = (uint8_t const *)send;
                crc.HL = crc16(0, p, 6);
                send[6] = crc.H;
                send[7] = crc.L;
            
                // スロットル送信
                //uint16_t pw = rsv[18];
                //pw = pw * 63 + 3936;
                //data1.pwm[1] = pw;

                //pw = rsv[19];
                //pw = pw * 63 + 3936;
                //data1.pwm[2] = pw;

                // 旋回送信
                //pw = rsv[20];
                //pw = pw * 63 + 3936;
                //data2.pwm[1] = pw; // 前ステアリング
                //pw = rsv[20];
                //pw = pw * 32 + 7904;
                //data2.pwm[0] = pw; // 後ステアリング
            }
            else if (mode == 3) { // 受信失敗
                for (i=0; i<RSVA_BYTES; i++) {
                    rsv[i] = 0;
                }
                id = 0;
                WiFi = 1;
            }

            // 主電源基板とSPI通信
            num_batt ++;
            sum_batt += (unsigned long)battery;
            spi_send();
            num_batt ++;
            sum_batt += (unsigned long)battery;
       
            // TWE LITE モジュールと通信
            uint8_t *cp = (uint8_t *)buf;
            cp[0] = 0x00; // 親機あて
            cp[1] = 0xA0; // 拡張形式
            cp[2] = id;   // 応答ID
            cp[3] = 0xFF; // オプション無し
            cp[4] = WiFi; // 受信感度

            HL16 v; // バッテリー電圧
            uint16_t vv = (uint16_t)(((long)10209 * (sum_batt / num_batt)) >> 12); // 1mV単位に変換 キャリブレーションする
            v.HL = (vv / 10); // 0.01V単位に変換
            if ((vv % 10) >= 5) v.HL ++; // 四捨五入
            cp[5] = v.H;
            cp[6] = v.L;
            v.HL = fired; // 射撃弾数
            if (err1) {
                v.HL = 65535;
            }
            cp[7] = v.H;
            cp[8] = v.L;
            cp[9] = data[0]; // 温度
            TWE_send(10, cp);

            // 俯仰
            uint8_t val = rsv[21];
            if (val > 128) { // ↑に
                cann1 -= table_pwm[val - 128];
                if (cann1 < MIN_CANNON) {
                    cann1 = MIN_CANNON;
                }
            }
            if (val < 128) { // ↓に
                cann1 += table_pwm[128 - val];
                if (cann1 > MAX_CANNON) {
                    cann1 = MAX_CANNON;
                }
            }
            PDC3 = 6542 + (cann1 >> 8);

            // トリガー状態確認
            trigger = (rsv[15] & 32);
            if (trigger) { // トリガー押された
                if (cnt_trig_off >= MIN_TRG_OFF) { // 直前まで十分な連続オフ回数あり
                    trigger1 = 1; // トリガー入力有効
                }
                cnt_trig_off = 0;
            }
            else {
                cnt_trig_off ++;
                if (cnt_trig_off > MIN_TRG_OFF) cnt_trig_off = MIN_TRG_OFF;
            }
            mode0 = mode;
            mode = 1;
        }

        if (trigger1) {; // トリガー入力有効
            if (in_fire == 0) {
                if (err1 == 0) {
                    in_fire = 1;
                    TMR2 = 0; // 0.3秒
                    TMR2_Start();
                }
            }
        }
        
        if (in_fire) {
            FIRE_SetHigh(); // トリガーON
        }

        if (cnt_mag) {
            cnt_mag --;
            if (rsv[15] & 16) {
                MAG_SetLow();
            }
            else {
                MAG_SetHigh();
            }
        }
        else {
            MAG_SetLow();
        }
        
#if (USE_LCD)
        LCD_i2C_cmd(0xC0);
        sprintf(buf, "%4d%4d%4d", trigger, cnt_trig_off, in_fire);
        LCD_i2C_data(buf);
#endif
        
    }    
    return 1; 
}
/**
 End of File
*/

