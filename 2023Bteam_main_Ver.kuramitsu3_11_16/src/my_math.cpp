/*
 * my_math.cpp
 *
 *  Created on: 2018/06/15
 *      Author: masato
 */

#include "mbed.h"
#include "my_math.hpp"

uint16_t fatan2(int8_t x, int8_t y) {
	uint8_t temp = atan2_table[MABS(y)][MABS(x)];

	if (x < 0) {
		if (y < 0)	//3
			return (180 + temp);
		else		//2
			return (180 - temp);
	}
	else {		//4
		if (y < 0)
			return (360 - temp);
		else		//1
			return temp;
	}
	return -1;
}

float myfsin(int16_t a){
	int8_t sign = 1;

	if(a < 0){
		sign = -1;
		a *= -1;
	}

	while(a > 360){
		a -= 360;
	}

	if(a <= 90){
		return sign*sin_table[a];
	}else if(a <= 180){
		return sign*sin_table[180-a];
	}else if(a <= 270){
		return -sign*sin_table[a-180];
	}else{
		return -sign*sin_table[360-a];
	}

}

float myfcos(int a){
	return myfsin(a + 90);
}

uint16_t mysqrt(uint16_t x,uint8_t nMAX){
	if (x <= 0){
		return 0;                 //x�����̐��̎�:0��Ԃ�
	}
	uint16_t t = x;
	for (int n = 1; n <= nMAX; n++){
		t = (t + x / t) / 2;
	}
	return t;
}


