#ifndef MICROMOUSE_HPP
#define MICROMOUSE_HPP

#include <iostream>

#define _interface struct

_interface Micromouse
{
    virtual void main_task() = 0;
};

#endif // MICROMOUSE_HPP