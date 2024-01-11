#include "include/Micromouse/UI/log.hpp"

void Log::ptr_by_sensor(t_sens_data *_sens) { sen = _sens; }

void Log::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Log::ptr_by_control(t_control *_control) { control = _control; }

void Log::ptr_by_map(t_map *_map) { map = _map; }

void Log::main_task()
{
    std::cout << "Log" << std::endl;
}

void Log1::ptr_by_sensor(t_sens_data *_sens) { sen = _sens; }

void Log1::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Log1::ptr_by_control(t_control *_control) { control = _control; }

void Log1::ptr_by_map(t_map *_map) { map = _map; }

void Log1::main_task()
{
    std::cout << "Log1" << std::endl;
}