
/*
    cytron_md.hpp
    Created on: 2021/09/18
    Author: Nakamura

    Cytron MD10C R3 などのヘッダファイル
*/

#include <mbed.h>
#define MAXOUTPUT 90

class CytronMD{
    private:
    DigitalOut _dir;
    PwmOut _pwm;
    Ticker _tick;
    bool _usetick;
    bool _continuous;
    int8_t _nowpower;
    int8_t _goalpower;
    void outputControl(int8_t _input){
        if(abs(_input)>MAXOUTPUT){
            if(_input<0) _input = -MAXOUTPUT;
            else         _input = MAXOUTPUT;
        }
        if(_input<0) _dir = 1;
        else         _dir = 0;
        _pwm = float(abs(_input))/100.0;
    }
    void trapControl(){
        _nowpower += (_nowpower < _goalpower) ? 1 : (_nowpower > _goalpower) ? -1 : 0;
        outputControl(_nowpower);
        if(!_continuous){
            if(_nowpower == _goalpower){
                _tick.detach();
                _usetick = 0;
            }
        }
    }

    public:
    CytronMD(PinName pwm_pin,PinName io_pin):
    _dir(io_pin),_pwm(pwm_pin){
        _pwm.period_us(200);    // 5kHz
        _nowpower = 0;
        _goalpower = 0;
    }
    void output_raw(int8_t _power){
        _nowpower = _power;
        _goalpower = _power;
        outputControl(_nowpower);
    }
    void output_trap(int8_t _power, float _delta = 0.02){
        _goalpower = _power;
        if(!_usetick){
            _tick.attach(callback(this,&CytronMD::trapControl),_delta);
            _usetick = 1;
        }
    }
    void tick_start(float _delta = 0.02){
        if(!_usetick){
            _tick.attach(callback(this,&CytronMD::trapControl),_delta);
            _continuous = 1;
            _usetick = 1;
        }
    }
    void tick_stop(){
        if(_usetick){
            _tick.detach();
            _continuous = 0;
            _usetick = 0;
        }
    }
    int8_t read(){
        return _nowpower;
    }
};
