#ifndef PCA9632_HPP
#define PCA9632_HPP

/*#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"*/
#include <stdexcept>
#include <iostream>
#include <math.h>
#include "sensor.hpp"


class PCA9632 : public Sensor{
    public:
        PCA9632();
        ~PCA9632();
        void init_i2c(i2c_port_t port, uint8_t adrs) override;
        void blink();
        void set(uint8_t led);
        void setmode(uint8_t mode);
    private:
        bool _init = false;
        i2c_port_t _port;
        uint8_t _adrs;
        uint8_t read(uint8_t reg) override;
        uint8_t write(uint8_t reg, uint8_t data) override;
        float speed;
        uint8_t mode;
};

#endif // PCA9632_HPP