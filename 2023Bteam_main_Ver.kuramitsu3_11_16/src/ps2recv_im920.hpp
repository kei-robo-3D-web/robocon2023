
/*
    ps2recv_im920.hpp
    Created on: 2022/08/31
    Author: Nakamura

    新ps2conからのim920を用いた受信用ヘッダファイル
*/

// example of functions
#if 0
// f303k8
#include <mbed.h>
#include "ps2recv_im920.hpp"
// im920_functions_v2.hppはsrcに置くだけでよい
Serial ps2(A7 ,A2, BAUD);
void Rx(){
  if(ps2.readable()){
    char getdata[RXDATASIZE] = {0};
    ps2.gets(getdata,RXDATASIZE);
    RXDT(getdata);
    ps2recv_decode();   // 受信データ解析
  }
}
int main() {
  ps2.attach(&Rx,Serial::RxIrq); // 受信割り込み
  while(1) {
    // 自分のコードを書く
    wait_us(1000); // 時間は自由
  }
}
#endif


#include <mbed.h>
#include "im920_functions_v2.hpp"   // インクルードを忘れない

// 受信データネーム
enum dataname{
    RIGHTBUTTONS = 0,
    LEFTBUTTONS,
    STICKRX,
    STICKRY,
    STICKLX,
    STICKLY,
    CHECKBIT,
    NON
};

// 受信データ
int ps2buttons[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 127, 127, 127, 60};
uint8_t checkbitsum = 0;    // 受信データチェックビット
uint8_t checkbiterror = 0;  // チェックビット比較結果 0:OK 1:NG(error)
typedef unsigned char uchar;

// RIGHTBUTTONS
#define CIRCLE   ps2buttons[ 0]
#define CROSS    ps2buttons[ 1]
#define SQUARE   ps2buttons[ 2]
#define TRIANGLE ps2buttons[ 3]
#define START    ps2buttons[ 4]
#define R1       ps2buttons[ 5]
#define R2       ps2buttons[ 6]
#define R3       ps2buttons[ 7]
// LEFTBUTTONS
#define UP       ps2buttons[ 8]
#define DOWN     ps2buttons[ 9]
#define RIGHT    ps2buttons[10]
#define LEFT     ps2buttons[11]
#define SELECT   ps2buttons[12]
#define L1       ps2buttons[13]
#define L2       ps2buttons[14]
#define L3       ps2buttons[15]
// STICKS
#define RIGHTX   ps2buttons[16]
#define RIGHTY   ps2buttons[17]
#define LEFTX    ps2buttons[18]
#define LEFTY    ps2buttons[19]
// CHECKBITS
#define CHECK    ps2buttons[20]

// 各bitデータをByteに合成(チェックビット用)
uchar bit2byte(bool bit7, bool bit6, bool bit5, bool bit4, bool bit3, bool bit2, bool bit1, bool bit0){
    return(128*bit7 + 64*bit6 + 32*bit5 + 16*bit4 + 8*bit3 + 4*bit2 + 2*bit1 + bit0);
}

void ps2recv_decode(){
    // データの取り出し
    CIRCLE    = (getdataset.dec[RIGHTBUTTONS]&1)/1;
    CROSS     = (getdataset.dec[RIGHTBUTTONS]&2)/2;
    SQUARE    = (getdataset.dec[RIGHTBUTTONS]&4)/4;
    TRIANGLE  = (getdataset.dec[RIGHTBUTTONS]&8)/8;
    START     = (getdataset.dec[RIGHTBUTTONS]&16)/16;
    R1        = (getdataset.dec[RIGHTBUTTONS]&32)/32;
    R2        = (getdataset.dec[RIGHTBUTTONS]&64)/64;
    R3        = (getdataset.dec[RIGHTBUTTONS]&128)/128;
    UP        = (getdataset.dec[LEFTBUTTONS]&1)/1;
    DOWN      = (getdataset.dec[LEFTBUTTONS]&2)/2;
    RIGHT     = (getdataset.dec[LEFTBUTTONS]&4)/4;
    LEFT      = (getdataset.dec[LEFTBUTTONS]&8)/8;
    SELECT    = (getdataset.dec[LEFTBUTTONS]&16)/16;
    L1        = (getdataset.dec[LEFTBUTTONS]&32)/32;
    L2        = (getdataset.dec[LEFTBUTTONS]&64)/64;
    L3        = (getdataset.dec[LEFTBUTTONS]&128)/128;
    RIGHTX    = getdataset.dec[STICKRX];
    RIGHTY    = getdataset.dec[STICKRY];
    LEFTX     = getdataset.dec[STICKLX];
    LEFTY     = getdataset.dec[STICKLY];
    CHECK     = getdataset.dec[CHECKBIT];

    // チェックビットの計算と判定
    checkbitsum = bit2byte( 0,
                            0,
                            ps2buttons[19]%2,
                            ps2buttons[18]%2,
                            ps2buttons[17]%2,
                            ps2buttons[16]%2,
                            (L3+L2+L1+SELECT+LEFT+RIGHT+DOWN+UP)%2,
                            (R3+R2+R1+START+TRIANGLE+SQUARE+CROSS+CIRCLE)%2);
    checkbiterror = (CHECK == checkbitsum ? 0 : 1);

    // エラーが起こった場合各データを初期化する．
    if(checkbiterror){
        memset(ps2buttons, 0, sizeof(ps2buttons));
        RIGHTX = 127;
        RIGHTY = 127;
        LEFTX  = 127;
        LEFTY  = 127;
        CHECK  =  60;
    }
}
