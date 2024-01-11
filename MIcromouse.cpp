#include "include/Micromouse/Micromouse.hpp"
#include "include/Micromouse/interrupt.hpp"

/* 基本的に全ての処理のをここにまとめ、mainで呼び出す。 */

void Micromouse()
{
    ADC adc(LED_FR, LED_FL, LED_R, LED_L, VBATT_CHANNEL);
    AS5047P enc_R(SPI3_HOST, ENC_CS_R);
    AS5047P enc_L(SPI3_HOST, ENC_CS_L);
    BUZZER buzzer(BUZZER_CH, BUZZER_TIMER, BUZZER_PIN);
    MPU6500 imu(SPI2_HOST, IMU_CS);
    PCA9632 led(I2C_NUM_0, LED_ADRS);
    Motor motor(BDC_R_MCPWM_GPIO_PH, BDC_R_MCPWM_GPIO_EN, BDC_L_MCPWM_GPIO_PH, BDC_L_MCPWM_GPIO_EN, FAN_PIN);

    Interrupt *interrupt;

    interrupt->set_module(adc,enc_R,enc_L,buzzer,imu,led,motor);

    

}
