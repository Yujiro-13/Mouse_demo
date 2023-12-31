#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include <iostream>

typedef struct
{
    uint8_t port;
} i2c_port_t;

typedef struct
{
    uint8_t bus;
} spi_host_device_t;

typedef struct 
{
    uint8_t bus;
}spi_device_handle_t;


typedef struct
{
    uint8_t cs;
} gpio_num_t;

typedef struct
{
    uint8_t err;
}esp_err_t;

typedef struct
{
    uint8_t TRUE;
    uint8_t FALSE;
}t_bool;  

typedef struct
{
    uint8_t dir;

}t_direction;

typedef struct 
{
    uint8_t _channel;
}ledc_channel_t;

typedef struct 
{
    uint8_t _timer;
}ledc_timer_t;

typedef struct
{
    uint8_t _channel;
}adc_channel_t;

typedef enum {
    GPIO_NUM_NC = -1,    /*!< Use to signal not connected to S/W */
    GPIO_NUM_0 = 0,     /*!< GPIO0, input and output */
    GPIO_NUM_1 = 1,     /*!< GPIO1, input and output */
    GPIO_NUM_2 = 2,     /*!< GPIO2, input and output */
    GPIO_NUM_3 = 3,     /*!< GPIO3, input and output */
    GPIO_NUM_4 = 4,     /*!< GPIO4, input and output */
    GPIO_NUM_5 = 5,     /*!< GPIO5, input and output */
    GPIO_NUM_6 = 6,     /*!< GPIO6, input and output */
    GPIO_NUM_7 = 7,     /*!< GPIO7, input and output */
    GPIO_NUM_8 = 8,     /*!< GPIO8, input and output */
    GPIO_NUM_9 = 9,     /*!< GPIO9, input and output */
    GPIO_NUM_10 = 10,   /*!< GPIO10, input and output */
    GPIO_NUM_11 = 11,   /*!< GPIO11, input and output */
    GPIO_NUM_12 = 12,   /*!< GPIO12, input and output */
    GPIO_NUM_13 = 13,   /*!< GPIO13, input and output */
    GPIO_NUM_14 = 14,   /*!< GPIO14, input and output */
    GPIO_NUM_15 = 15,   /*!< GPIO15, input and output */
    GPIO_NUM_16 = 16,   /*!< GPIO16, input and output */
    GPIO_NUM_17 = 17,   /*!< GPIO17, input and output */
    GPIO_NUM_18 = 18,   /*!< GPIO18, input and output */
    GPIO_NUM_19 = 19,   /*!< GPIO19, input and output */
    GPIO_NUM_20 = 20,   /*!< GPIO20, input and output */
    GPIO_NUM_21 = 21,   /*!< GPIO21, input and output */
    GPIO_NUM_26 = 26,   /*!< GPIO26, input and output */
    GPIO_NUM_27 = 27,   /*!< GPIO27, input and output */
    GPIO_NUM_28 = 28,   /*!< GPIO28, input and output */
    GPIO_NUM_29 = 29,   /*!< GPIO29, input and output */
    GPIO_NUM_30 = 30,   /*!< GPIO30, input and output */
    GPIO_NUM_31 = 31,   /*!< GPIO31, input and output */
    GPIO_NUM_32 = 32,   /*!< GPIO32, input and output */
    GPIO_NUM_33 = 33,   /*!< GPIO33, input and output */
    GPIO_NUM_34 = 34,   /*!< GPIO34, input and output */
    GPIO_NUM_35 = 35,   /*!< GPIO35, input and output */
    GPIO_NUM_36 = 36,   /*!< GPIO36, input and output */
    GPIO_NUM_37 = 37,   /*!< GPIO37, input and output */
    GPIO_NUM_38 = 38,   /*!< GPIO38, input and output */
    GPIO_NUM_39 = 39,   /*!< GPIO39, input and output */
    GPIO_NUM_40 = 40,   /*!< GPIO40, input and output */
    GPIO_NUM_41 = 41,   /*!< GPIO41, input and output */
    GPIO_NUM_42 = 42,   /*!< GPIO42, input and output */
    GPIO_NUM_43 = 43,   /*!< GPIO43, input and output */
    GPIO_NUM_44 = 44,   /*!< GPIO44, input and output */
    GPIO_NUM_45 = 45,   /*!< GPIO45, input and output */
    GPIO_NUM_46 = 46,   /*!< GPIO46, input and output */
    GPIO_NUM_47 = 47,   /*!< GPIO47, input and output */
    GPIO_NUM_48 = 48,   /*!< GPIO48, input and output */
    GPIO_NUM_MAX,
/** @endcond */
} gpio_num_t;

typedef enum {
    LEDC_CHANNEL_0 = 0, /*!< LEDC channel 0 */
    LEDC_CHANNEL_1,     /*!< LEDC channel 1 */
    LEDC_CHANNEL_2,     /*!< LEDC channel 2 */
    LEDC_CHANNEL_3,     /*!< LEDC channel 3 */
    LEDC_CHANNEL_4,     /*!< LEDC channel 4 */
    LEDC_CHANNEL_5,     /*!< LEDC channel 5 */
#if SOC_LEDC_CHANNEL_NUM > 6
    LEDC_CHANNEL_6,     /*!< LEDC channel 6 */
    LEDC_CHANNEL_7,     /*!< LEDC channel 7 */
#endif
    LEDC_CHANNEL_MAX,
} ledc_channel_t;

typedef enum {
    LEDC_TIMER_0 = 0, /*!< LEDC timer 0 */
    LEDC_TIMER_1,     /*!< LEDC timer 1 */
    LEDC_TIMER_2,     /*!< LEDC timer 2 */
    LEDC_TIMER_3,     /*!< LEDC timer 3 */
    LEDC_TIMER_MAX,
} ledc_timer_t;

struct spi_transaction_t {
    uint32_t flags;                 ///< Bitwise OR of SPI_TRANS_* flags
    uint16_t cmd;                   /**< Command data, of which the length is set in the ``command_bits`` of spi_device_interface_config_t.
                                      *
                                      *  <b>NOTE: this field, used to be "command" in ESP-IDF 2.1 and before, is re-written to be used in a new way in ESP-IDF 3.0.</b>
                                      *
                                      *  Example: write 0x0123 and command_bits=12 to send command 0x12, 0x3_ (in previous version, you may have to write 0x3_12).
                                      */
    uint64_t addr;                  /**< Address data, of which the length is set in the ``address_bits`` of spi_device_interface_config_t.
                                      *
                                      *  <b>NOTE: this field, used to be "address" in ESP-IDF 2.1 and before, is re-written to be used in a new way in ESP-IDF3.0.</b>
                                      *
                                      *  Example: write 0x123400 and address_bits=24 to send address of 0x12, 0x34, 0x00 (in previous version, you may have to write 0x12340000).
                                      */
    size_t length;                  ///< Total data length, in bits
    size_t rxlength;                ///< Total data length received, should be not greater than ``length`` in full-duplex mode (0 defaults this to the value of ``length``).
    void *user;                     ///< User-defined variable. Can be used to store eg transaction ID.
    union {
        const void *tx_buffer;      ///< Pointer to transmit buffer, or NULL for no MOSI phase
        uint8_t tx_data[4];         ///< If SPI_TRANS_USE_TXDATA is set, data set here is sent directly from this variable.
    };
    union {
        void *rx_buffer;            ///< Pointer to receive buffer, or NULL for no MISO phase. Written by 4 bytes-unit if DMA is used.
        uint8_t rx_data[4];         ///< If SPI_TRANS_USE_RXDATA is set, data is received directly to this variable
    };
} ; 

typedef struct {
    uint8_t command_bits;           ///< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
    uint8_t address_bits;           ///< Default amount of bits in address phase (0-64), used when ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored.
    uint8_t dummy_bits;             ///< Amount of dummy bits to insert between address and data phase
    uint8_t mode;                   /**< SPI mode, representing a pair of (CPOL, CPHA) configuration:
                                         - 0: (0, 0)
                                         - 1: (0, 1)
                                         - 2: (1, 0)
                                         - 3: (1, 1)
                                     */
    /*spi_clock_source_t clock_source;*////< Select SPI clock source, `SPI_CLK_SRC_DEFAULT` by default.
    uint16_t duty_cycle_pos;        ///< Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    uint16_t cs_ena_pretrans;       ///< Amount of SPI bit-cycles the cs should be activated before the transmission (0-16). This only works on half-duplex transactions.
    uint8_t cs_ena_posttrans;       ///< Amount of SPI bit-cycles the cs should stay active after the transmission (0-16)
    int clock_speed_hz;             ///< Clock speed, divisors of the SPI `clock_source`, in Hz
    int input_delay_ns;             /**< Maximum data valid time of slave. The time required between SCLK and MISO
        valid, including the possible clock delay from slave to master. The driver uses this value to give an extra
        delay before the MISO is ready on the line. Leave at 0 unless you know you need a delay. For better timing
        performance at high frequency (over 8MHz), it's suggest to have the right value.
        */
    int spics_io_num;               ///< CS GPIO pin for this device, or -1 if not used
    uint32_t flags;                 ///< Bitwise OR of SPI_DEVICE_* flags
    int queue_size;                 ///< Transaction queue size. This sets how many transactions can be 'in the air' (queued using spi_device_queue_trans but not yet finished using spi_device_get_trans_result) at the same time
    /*transaction_cb_t pre_cb;*/   /**< Callback to be called before a transmission is started.
                                 *
                                 *  This callback is called within interrupt
                                 *  context should be in IRAM for best
                                 *  performance, see "Transferring Speed"
                                 *  section in the SPI Master documentation for
                                 *  full details. If not, the callback may crash
                                 *  during flash operation when the driver is
                                 *  initialized with ESP_INTR_FLAG_IRAM.
                                 */
    /*transaction_cb_t post_cb;*/  /**< Callback to be called after a transmission has completed.
                                 *
                                 *  This callback is called within interrupt
                                 *  context should be in IRAM for best
                                 *  performance, see "Transferring Speed"
                                 *  section in the SPI Master documentation for
                                 *  full details. If not, the callback may crash
                                 *  during flash operation when the driver is
                                 *  initialized with ESP_INTR_FLAG_IRAM.
                                 */
} spi_device_interface_config_t;

typedef struct {
    union {
      int mosi_io_num;    ///< GPIO pin for Master Out Slave In (=spi_d) signal, or -1 if not used.
      int data0_io_num;   ///< GPIO pin for spi data0 signal in quad/octal mode, or -1 if not used.
    };
    union {
      int miso_io_num;    ///< GPIO pin for Master In Slave Out (=spi_q) signal, or -1 if not used.
      int data1_io_num;   ///< GPIO pin for spi data1 signal in quad/octal mode, or -1 if not used.
    };
    int sclk_io_num;      ///< GPIO pin for SPI Clock signal, or -1 if not used.
    union {
      int quadwp_io_num;  ///< GPIO pin for WP (Write Protect) signal, or -1 if not used.
      int data2_io_num;   ///< GPIO pin for spi data2 signal in quad/octal mode, or -1 if not used.
    };
    union {
      int quadhd_io_num;  ///< GPIO pin for HD (Hold) signal, or -1 if not used.
      int data3_io_num;   ///< GPIO pin for spi data3 signal in quad/octal mode, or -1 if not used.
    };
    int data4_io_num;     ///< GPIO pin for spi data4 signal in octal mode, or -1 if not used.
    int data5_io_num;     ///< GPIO pin for spi data5 signal in octal mode, or -1 if not used.
    int data6_io_num;     ///< GPIO pin for spi data6 signal in octal mode, or -1 if not used.
    int data7_io_num;     ///< GPIO pin for spi data7 signal in octal mode, or -1 if not used.
    int max_transfer_sz;  ///< Maximum transfer size, in bytes. Defaults to 4092 if 0 when DMA enabled, or to `SOC_SPI_MAXIMUM_BUFFER_SIZE` if DMA is disabled.
    uint32_t flags;       ///< Abilities of bus to be checked by the driver. Or-ed value of ``SPICOMMON_BUSFLAG_*`` flags.
    /*intr_cpu_id_t  isr_cpu_id;*/    ///< Select cpu core to register SPI ISR.
    int intr_flags;       /**< Interrupt flag for the bus to set the priority, and IRAM attribute, see
                           *  ``esp_intr_alloc.h``. Note that the EDGE, INTRDISABLED attribute are ignored
                           *  by the driver. Note that if ESP_INTR_FLAG_IRAM is set, ALL the callbacks of
                           *  the driver, and their callee functions, should be put in the IRAM.
                           */
} spi_bus_config_t;

typedef enum {
    ADC_CHANNEL_0,     ///< ADC channel
    ADC_CHANNEL_1,     ///< ADC channel
    ADC_CHANNEL_2,     ///< ADC channel
    ADC_CHANNEL_3,     ///< ADC channel
    ADC_CHANNEL_4,     ///< ADC channel
    ADC_CHANNEL_5,     ///< ADC channel
    ADC_CHANNEL_6,     ///< ADC channel
    ADC_CHANNEL_7,     ///< ADC channel
    ADC_CHANNEL_8,     ///< ADC channel
    ADC_CHANNEL_9,     ///< ADC channel
} adc_channel_t;

typedef enum {
//SPI1 can be used as GPSPI only on ESP32
    SPI1_HOST=0,    ///< SPI1
    SPI2_HOST=1,    ///< SPI2
//#if SOC_SPI_PERIPH_NUM > 2
    SPI3_HOST=2,    ///< SPI3
//#endif
    SPI_HOST_MAX,   ///< invalid host value
} spi_host_device_t;

typedef enum {
    I2C_NUM_0 = 0,              /*!< I2C port 0 */
#if SOC_I2C_NUM >= 2
    I2C_NUM_1,                  /*!< I2C port 1 */
#endif /* SOC_I2C_NUM >= 2 */
#if SOC_LP_I2C_NUM >= 1
    LP_I2C_NUM_0,               /*< LP_I2C port 0 */
#endif /* SOC_LP_I2C_NUM >= 1 */
    I2C_NUM_MAX,                /*!< I2C port max */
} i2c_port_t;



#endif // STRUCTS_HPP