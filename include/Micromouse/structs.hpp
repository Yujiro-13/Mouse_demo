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
#endif // STRUCTS_HPP