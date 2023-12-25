#ifndef UI_HPP
#define UI_HPP

#define _interface struct

_interface UI
{
    virtual void main_task() = 0;
};

void call_task();
void set_task();
void get_task();
void set_mode();

#endif // UI_HPP