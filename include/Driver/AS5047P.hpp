#ifndef AS5047P_HPP
#define AS5047P_HPP

/*#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"*/
#include <cstring>
#include "sensor.hpp"

#define AS5047P_WHO_AM_I 0x70
#define AS5047P_READ_FLAG 0x4000
#define ENC_MAX 16384
#define ENC_HALF 8192

class AS5047P : public Sensor{
    public:
        AS5047P();
        ~AS5047P();

        void init_spi(spi_host_device_t bus, gpio_num_t cs) override;
        uint16_t readAngle();
        void ShowAngle();
    private:
        uint16_t read16(uint8_t reg) override;
        gpio_num_t _cs;
        uint8_t CalcParity(uint16_t data);
        spi_device_handle_t _spi;
};

#endif