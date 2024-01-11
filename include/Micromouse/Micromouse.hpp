#ifndef MICROMOUSE_HPP
#define MICROMOUSE_HPP

#include <iostream>
#include "structs.hpp"

#define Interface class

Interface Micromouse
{
    virtual void ptr_by_sensor(t_sens_data *_sens) = 0;
    virtual void ptr_by_motion(t_mouse_motion_val *_val) = 0;
    virtual void ptr_by_control(t_control *_control) = 0;
    virtual void ptr_by_map(t_map *_map) = 0;
};

void Micromouse(){}



#endif // MICROMOUSE_HPP