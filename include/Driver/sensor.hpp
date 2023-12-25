#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <iostream>
#include "structs.hpp"

#define _driver struct

_driver Sensor // センサドライバのインターフェースクラス
{
    virtual void init_i2c(i2c_port_t port, uint8_t adrs) = 0;
    virtual void init_spi(spi_host_device_t bus, gpio_num_t cs) = 0;
    virtual uint8_t read(uint8_t reg) = 0;
    virtual uint8_t write(uint8_t reg, uint8_t data) = 0;
    virtual uint16_t read16(uint8_t reg) = 0;
    virtual uint8_t whoami() = 0;
};

#endif // SENSOR_HPP