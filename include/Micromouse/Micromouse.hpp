#ifndef MICROMOUSE_HPP
#define MICROMOUSE_HPP

#include <iostream>
#include <vector>
#include <memory>
#include "structs.hpp"
#include "adc.hpp"
#include "AS5047P.hpp"
#include "Buzzer.hpp"
#include "Motor.hpp"
#include "MPU6500.hpp"
#include "PCA9632.hpp"




#define Interface struct

Interface Micromouse
{
    virtual void ptr_by_sensor(t_sens_data *_sens) = 0;
    virtual void ptr_by_motion(t_mouse_motion_val *_val) = 0;
    virtual void ptr_by_control(t_control *_control) = 0;
    virtual void ptr_by_map(t_map *_map) = 0;
    virtual void set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) = 0;
};

void MICROMOUSE(){}



#endif // MICROMOUSE_HPP