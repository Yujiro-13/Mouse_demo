#ifndef LOG_HPP
#define LOG_HPP

#include <iostream>
#include "UI.hpp"

class Log : public UI
{
    void main_task() override;
};

class Log1 : public UI
{
    void main_task() override;
};

#endif // LOG_HPP