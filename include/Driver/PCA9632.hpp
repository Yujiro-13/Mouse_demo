#ifndef PCA9632_HPP
#define PCA9632_HPP

/*#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"*/
#include <stdexcept>
#include <iostream>
#include <math.h>
#include "sensor.hpp"

class PCA9632
{
public:
    PCA9632(i2c_port_t port, uint8_t adrs);
    ~PCA9632();
    void blink();
    void set(uint8_t led);
    void setmode(uint8_t mode);

private:
    bool _init = false;
    i2c_port_t _port;
    uint8_t _adrs;
    uint8_t read(uint8_t reg);
    uint8_t write(uint8_t reg, uint8_t data);
    float speed;
    uint8_t mode;
    gpio_num_t LED_SDA = GPIO_NUM_37;
    gpio_num_t LED_SCL = GPIO_NUM_38;
    uint LED_FREQ = 1000 * 1000;
    uint8_t LED_ADRS = 0x62;
    i2c_config_t i2c_conf;
};

#endif // PCA9632_HPP