//9月7日　足回りにジャイロセンサを追加

#include <mbed.h>
//以下4つのヘッダファイルは独自制作であり、著作権の関係上src内には含めない
//そのため、独自の関数には逐一解説を付ける
#include "f401_mdd.hpp"
#include "cytron_md.hpp"
#include "ps2recv_im920.hpp"
#include "my_math.hpp"

#define EDFMIN 1035
#define EDFOP  1085
#define EDFMAX 500
#define EDFWAIT 500000

CytronMD motorRF(PWM_1, IO_1);//モータで使用するPWMピンとIOピンを宣言
CytronMD motorLF(PWM_2, IO_2);
CytronMD motorRB(PWM_3, IO_3);
CytronMD motorLB(PWM_4, IO_4);
CytronMD edfAngle(PWM_5, IO_5);
DigitalOut airFront(AIR_1);
DigitalOut airBack(AIR_2);
DigitalOut airCenter(AIR_3);
DigitalOut gyroReset(PA_6);
PwmOut green(PB_15);
PwmOut blue(PB_0);
swSampling edfAngleMin(PB_12);//サンプリングし、チャタリング対策を行うIOピンを宣言
PwmOut edfFan(UnitIO2);
Serial im920(UART1_TX ,UART1_RX, BAUD);//無線通信(920MHz帯を扱うim920)の宣言
Serial gyro(UART3_TX, UART3_RX, 115200);
Ticker angle;
Ticker angleSample;
Ticker edfUp;
Ticker edfDown;
Ticker swsamp_tic;
Timer edfWait;

/*EDF*/
volatile int edfPower = 0;
volatile int edfCon = 750;
float edfUpTime = 0.0067;
int anglePower = 0;
int waitCount = 0;
/*通信*/
uint8_t up_flag=0;
uint8_t down_flag=0;
uint8_t right_flag=0;
uint8_t left_flag=0;
uint8_t circle_flag=0;
uint8_t cross_flag=0;
uint8_t square_flag=0;
uint8_t triangle_flag=0;
uint8_t R1_flag=0;
uint8_t R2_flag=0;
uint8_t R3_flag=0;
uint8_t L1_flag=0;
uint8_t L2_flag=0;
uint8_t L3_flag=0;
uint8_t start_flag=0;
uint8_t select_flag=0;
volatile double stickRX = 0;
volatile double stickRY = 0;
volatile double stickLX = 0;
volatile double stickLY = 0;
/*ジャイロセンサ用*/
float arktan = 0;
float x = 0.0,y = 0.0;
float vec = 0.0;
uint8_t gyroRecive = 0;
int rad = 0;

int mflim_f = 0;
int flim_f = 0;
int mblim_f = 0;
int blim_f = 0;
int edf_down_limit = 0;
/*足回り*/
float motorRFpow = 0;
float motorLFpow = 0;
float motorRBpow = 0;
float motorLBpow = 0;
int maxwheel = 70;

//無線通信を行う関数
//コントローラ(PS2のコントローラを改造した物)からボタンの情報とアナログスティックの情報を取得
//ボタンの情報はCIRCLE(〇ボタン)やUP(上ボタン)などの変数に0or1で格納される
void Rx(){
    if(im920.readable()){
        char getdata[RXDATASIZE] = {0};
        im920.gets(getdata,RXDATASIZE);
        RXDT(getdata);
        ps2recv_decode();
    }
}

void edf_up(void){
    edfPower += 10;
    edfFan.pulsewidth_us(EDFMIN + edfPower);
    wait_us(1);
}

void edf_down(void){
    edfPower -= 10;
    edfFan.pulsewidth_us(EDFMIN + edfPower);
    wait_us(1);
}

void my_wait(void){
    waitCount += 1;
}

void gyro_RX(){
    gyroRecive = gyro.getc();
}

void Sampling(){
    edfAngleMin.sampling_NC();
}



int main(){
    im920.attach(&Rx, Serial::RxIrq);//無線を扱うためのシリアル通信の割り込み宣言
    gyro.attach(gyro_RX, Serial::RxIrq);
    angleSample.attach(&Sampling, 0.01);//サンプリングの割り込み宣言
    edfFan.period(0.02);
    edfPower = 0;
    airFront = 0;
    airBack = 0;
    airCenter = 0;

    blue = 1.0;
    green = 1.0;
    
    while(1){
        //角度調整
        if(R1 && !(R1_flag)){//R1ボタンが押されたとき
            R1_flag = 1;
            anglePower = 20;
        }else if(!(R1) && (R1_flag)){
            R1_flag = 0;
            anglePower = 0;
        }
        if(L1 && !(L1_flag)){//L1ボタンが押されたとき
            L1_flag = 1;
            anglePower = -20;
        }else if(!(L1) && (L1_flag)){
            L1_flag = 0;
            anglePower = 0;
        }

        //出力130%
        if((UP) && !(up_flag)){//上ボタンが押されたとき
            up_flag = 1; 
            edfCon = 750; 
            edfUpTime = 0.0067;
            blue = 1.0;
            green = 1.0;
        }else if(!(UP) && (up_flag)){
            up_flag = 0;
        }

        //出力100%
        if((LEFT) && !(down_flag)){//左ボタンが押されたとき
            down_flag = 1;
            edfCon = 650;
            edfUpTime = 0.0078;
            blue = 0.0;
            green = 1.0;
        }else if(!(LEFT) && (down_flag)){
            down_flag = 0;
        }

        //出力80%
        if((DOWN) && !(down_flag)){//下ボタンが押されたとき
            down_flag = 1;
            edfCon = 400; 
            edfUpTime = 0.0125;
            blue = 1.0;
            green = 0.0;
        }else if(!(DOWN) && (down_flag)){
            down_flag = 0;
        }

        //〇ボタンが押されたとき
        if(CIRCLE && !(circle_flag)){     //EDFを一瞬だけ出力
            circle_flag = 1;
            edfUp.attach(&edf_up,edfUpTime);
        }else if(!(CIRCLE) && (circle_flag)){
            circle_flag = 0;
        }

        if(edfPower >= edfCon){
            edfUp.detach();
            edfWait.start();
            edfPower = edfCon;
        }

        if(edfWait.read() >= 1.5){
            edfWait.stop();
            edfWait.reset();
            edfDown.attach(&edf_down,edfUpTime);
        }

        if(edfPower <= 0){
            edfDown.detach();
            edfPower = 0;
        }


        edfFan.pulsewidth_us(EDFMIN + edfPower);
        edfAngle.output_raw(anglePower);

        //△ボタンが押されたとき
        if((TRIANGLE) && !(triangle_flag)){ //補助キャスタ上げ下げ
            triangle_flag = 1;
            airCenter = !(airCenter);
        }else if(!(TRIANGLE) && (triangle_flag)){
            triangle_flag = 0;
        }
        //R2ボタンが押されたとき
        if((R2) && !(R2_flag)){ //足回りエアシリンダ前輪
            R2_flag = 1;
            airFront = 1;

        }else if((R2) && (R2_flag)){
            if(edfAngleMin.state != 1){
                anglePower = -10;
            }else if(edfAngleMin.state == 1){
                anglePower = 0;
            }
        }else if(!(R2) && (R2_flag)){
            anglePower = 0;
            R2_flag = 0;
            airFront = 0;
        }
        
        //L2ボタンが押されたとき
        if((L2) && !(L2_flag)){ //足回りエアシリンダ後輪
            L2_flag = 1;
            airBack = 1;

        }else if((L2) && (L2_flag)){
            if(edfAngleMin.state != 1){
                anglePower = -10;
                anglePower = -10;
            }else if(edfAngleMin.state == 1){
                anglePower = 0;
            }
        }else if(!(L2) && (L2_flag)){
            anglePower = 0;
            L2_flag = 0;
            airBack = 0;
        }

        // アナログボタンプッシュ前対策
        if((stickRX == 50) && (stickRY == -50)){
            stickRX = 0;
            stickRY = 0;
        }
        if((stickLX == 50) && (stickLY == -50)){
            stickLX = 0;
            stickLY = 0;
        }
        
        //SELECTボタンが押されたとき
        if((SELECT) && !(select_flag)){
            select_flag = 1;
            gyroReset = 1;
        }else if(!(SELECT) && (select_flag)){
            select_flag = 0;
            gyroReset = 0;
        }

        /*足回り*/
        stickRX = ((RIGHTX / 255.0) - 0.5) * 100;//右スティックのX軸の値を-50～50で取得
        stickRY = ((RIGHTY / 255.0) - 0.5) * 100;//右スティックのY軸の値を-50～50で取得
        stickLX = ((LEFTX  / 255.0) - 0.5) * 100;//左スティックのX軸の値を-50～50で取得

        arktan = fatan2(((int)stickRX),((int)stickRY));
        rad = ((gyroRecive) / 0.708);
        vec = mysqrt(((stickRX * stickRX) + (stickRY * stickRY)),10) / 50;//右スティックの傾きをベクトルで取得
        x = (vec * myfcos((int)rad + (int)arktan));//スティックの傾きのコサイン成分
        y = (vec * myfsin((int)rad + (int)arktan));//スティックの傾きのサイン成分

        if(stickLX == 0){
            motorRFpow = (+x - y);
            motorLFpow = (+x + y);
            motorRBpow = (-x - y);
            motorLBpow = (-x + y);
        }else if(stickRX == 0 && stickRY == 0){
            motorRFpow = stickLX / 2;
            motorLFpow = stickLX / 2;
            motorRBpow = stickLX / 2;
            motorLBpow = stickLX / 2;
        }

        //低速モード
        if((RIGHT) && !(right_flag)){//右ボタンが押されたとき
            right_flag = 1;
            maxwheel = 30;
        }else if(!(RIGHT) && (right_flag)){
            right_flag = 0;
            maxwheel = 70;
        }

        motorRF.output_raw(limit(motorRFpow * 100 * 1.1 , maxwheel));//宣言したモータに-maxwheel～maxwheelの範囲で出力
        motorLF.output_raw(limit(motorLFpow * 100       , maxwheel));
        motorRB.output_raw(limit(motorRBpow * 100 * 1.2 , maxwheel));
        motorLB.output_raw(limit(motorLBpow * 100 * 1.6 , maxwheel));

        wait_us(1000);
    }
}
