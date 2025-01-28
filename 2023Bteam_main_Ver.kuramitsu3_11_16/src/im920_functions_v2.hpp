
/*
    im920_functions_v2.hpp
    Created on: 2021/08/24
    Author: Nakamura

    easy_im920うまくいかなかったから
    変換関数のみ抜き出したヘッダを用意

    mainで用いる関数例を追加
*/

// example of functions
#if 0
Serial im920(D1,D0,BAUD); // 宣言
void Tx(uchar send){ // 改造すれば複数バイトも可
    char senddata[TXDATASIZE] = {0};
    TXDT(senddata,send,0,0,0,0,0,0,0);
    im920.puts(senddata);
    im920.putc(0x0d);
    im920.putc(0x0a);
}
void Rx(){ // 受信割り込み関数
    if(im920.readable()){
        char getdata[RXDATASIZE] = {0};
        im920.gets(getdata,RXDATASIZE);
        RXDT(getdata);
        // ここに受信処理
    }
}
im920.attach(&Rx,Serial::RxIrq); // 受信割り込み
#endif


#include <mbed.h>

#define BAUD 19200
#define GAP 0
#define RXDATASIZE 40
#define TXDATASIZE 21
typedef unsigned char uchar;

//tx_mode
char Dec2Ascii(uchar input){
    input %= 16;
    switch(input){
    case  0: input = '0'; break;
    case  1: input = '1'; break;
    case  2: input = '2'; break;
    case  3: input = '3'; break;
    case  4: input = '4'; break;
    case  5: input = '5'; break;
    case  6: input = '6'; break;
    case  7: input = '7'; break;
    case  8: input = '8'; break;
    case  9: input = '9'; break;
    case 10: input = 'A'; break;
    case 11: input = 'B'; break;
    case 12: input = 'C'; break;
    case 13: input = 'D'; break;
    case 14: input = 'E'; break;
    case 15: input = 'F'; break;
    }
    return input;
}
void Byte2Ascii(char* tx_up4bit, char* tx_down4bit, uchar input){
    *tx_up4bit = Dec2Ascii((input-(input%16))/16);
    *tx_down4bit = Dec2Ascii(input%16);
}
void TXDT(char* senddataset/*length=21*/,uchar dec1,uchar dec2,uchar dec3,uchar dec4,uchar dec5,uchar dec6,uchar dec7,uchar dec8){
    char data[17] = {0};
    Byte2Ascii(&data[ 0],&data[ 1],dec1);
    Byte2Ascii(&data[ 2],&data[ 3],dec2);
    Byte2Ascii(&data[ 4],&data[ 5],dec3);
    Byte2Ascii(&data[ 6],&data[ 7],dec4);
    Byte2Ascii(&data[ 8],&data[ 9],dec5);
    Byte2Ascii(&data[10],&data[11],dec6);
    Byte2Ascii(&data[12],&data[13],dec7);
    Byte2Ascii(&data[14],&data[15],dec8);
    sprintf(senddataset,"%s%s","TXDA",data);
}

//rx_mode
uchar Ascii2Dec(uchar input){
    switch(input){
        case 0x30: input =  0; break;
        case 0x31: input =  1; break;
        case 0x32: input =  2; break;
        case 0x33: input =  3; break;
        case 0x34: input =  4; break;
        case 0x35: input =  5; break;
        case 0x36: input =  6; break;
        case 0x37: input =  7; break;
        case 0x38: input =  8; break;
        case 0x39: input =  9; break;
        case 0x41: input = 10; break;
        case 0x42: input = 11; break;
        case 0x43: input = 12; break;
        case 0x44: input = 13; break;
        case 0x45: input = 14; break;
        case 0x46: input = 15; break;
        default  : input =  0; break;
    }
    return input;
}
uchar Ascii2Byte(uchar rx_up4bit, uchar rx_down4bit){
    return Ascii2Dec(rx_up4bit)*16+Ascii2Dec(rx_down4bit);
}
struct dataset{
    int node_number;
    int signal_strength;
    int dec[8];
}getdataset;
void RXDT(char* data){
    getdataset.node_number = Ascii2Byte(data[0 + GAP],data[1 + GAP]);
    getdataset.signal_strength = Ascii2Byte(data[8 + GAP],data[9 + GAP]);
    getdataset.dec[0] = Ascii2Byte(data[11 + GAP],data[12 + GAP]);
    getdataset.dec[1] = Ascii2Byte(data[14 + GAP],data[15 + GAP]);
    getdataset.dec[2] = Ascii2Byte(data[17 + GAP],data[18 + GAP]);
    getdataset.dec[3] = Ascii2Byte(data[20 + GAP],data[21 + GAP]);
    getdataset.dec[4] = Ascii2Byte(data[23 + GAP],data[24 + GAP]);
    getdataset.dec[5] = Ascii2Byte(data[26 + GAP],data[27 + GAP]);
    getdataset.dec[6] = Ascii2Byte(data[29 + GAP],data[30 + GAP]);
    getdataset.dec[7] = Ascii2Byte(data[32 + GAP],data[33 + GAP]);
}