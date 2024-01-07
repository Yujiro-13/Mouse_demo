#ifndef ADC_HPP
#define ADC_HPP

/*#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"*/
#include <iostream>
#include "peripheral.hpp"
#include "sensor.hpp"
#include "structs.hpp"

class ADC : public Sensor
{
public:
    ADC(IRLED_FR* led_FR, IRLED_FL* led_FL, IRLED_R* led_R, IRLED_L* led_L, adc_channel_t VBATT);
    ~ADC();

    void GetData() override;

private:
    int before[4];
    int sensors[4];
    void SetIRLED(uint8_t led, IRLED_FR& led_FR, IRLED_FL& led_FL, IRLED_R& led_R, IRLED_L& led_L);
    float BatteryVoltage();
    void ReadSensor(int* sensors,uint8_t mask);
    void WallSensor();
    IRLED_FL led_FL;
    IRLED_FR led_FR;
    IRLED_R led_R;
    IRLED_L led_L;
    
};

#endif