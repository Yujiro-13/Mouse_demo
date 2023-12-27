#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <iostream>
#include "structs.hpp"

#define _driver struct

_driver Sensor // センサドライバのインターフェースクラス
{
    virtual void GetData() = 0;
};

#endif // SENSOR_HPP