/*
f401_mdd_ver2.hpp
Created on: 2022/08/24
Author: Kiyota

2022年Aチーム用に調整
*/

#pragma once

#include <mbed.h>

//PWMピン
#define PWM_1 PB_3
#define PWM_2 PB_10
#define PWM_3 PB_9
#define PWM_4 PB_7
#define PWM_5 PA_5

//IOピン
#define IO_1 PA_0
#define IO_2 PA_1
#define IO_3 PC_7
#define IO_4 PB_6
#define IO_5 PA_7

//UnitIO
#define UnitIO1 PC_5
#define UnitIO2 PC_9

//AIR
#define AIR_1 PA_4
#define AIR_2 PA_8
#define AIR_3 PC_15

//通信
#define UART1_TX PA_9
#define UART1_RX PA_10
#define UART2_TX PA_2
#define UART2_RX PA_3
#define UART3_TX PA_11
#define UART3_RX PA_12

//チャタリング対策
class swSampling{
    private:
    DigitalIn digitalin;
    volatile uint8_t temp;
    volatile uint8_t prev;

    public:
    volatile uint8_t state;
    swSampling(PinName io_pin):
    digitalin(io_pin){
        digitalin.mode(PullUp);
        temp = 0;
        prev = 0;
    }
    inline void sampling_NC(){
        temp = digitalin.read();
        if(temp == prev)
            state = temp;
        prev = temp;
    }
    inline void sampling_NO(){
        temp = digitalin.read();
        if(temp == prev)
            state = !temp;
        prev = temp;
    }
};

//範囲制限
int limit(int val,int range){
    range = abs(range);
    if(-range>val)
        return -range;
    else if(range<val)
        return range;
    else
        return val;
}

//範囲丸め込み
double rounding(double val, double min, double max, double target){
    if(val > min && val < max)
        val = target;
    
    return val;
}