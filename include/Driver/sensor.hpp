#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <iostream>
#include "structs.hpp"


/*
    < センサドライバのインターフェースクラス >
    センサからデータを取得するドライバを作成する場合は、このクラスを継承
*/

#define _driver struct

_driver Sensor 
{
    virtual void GetData() = 0;
};

#endif // SENSOR_HPP