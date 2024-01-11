#include "include/Micromouse/UI/fast.hpp"

void Fast::ptr_by_sensor(t_sens_data *_sens) { sen = _sens; }

void Fast::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Fast::ptr_by_control(t_control *_control) { control = _control; }

void Fast::ptr_by_map(t_map *_map) { map = _map; }

void Fast::main_task()
{
    std::cout << "Fast" << std::endl;
}

void Fast2::ptr_by_sensor(t_sens_data *_sens) { sen = _sens; }

void Fast2::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Fast2::ptr_by_control(t_control *_control) { control = _control; }

void Fast2::ptr_by_map(t_map *_map) { map = _map; }

void Fast2::main_task()
{
    std::cout << "Fast2" << std::endl;
}

void Fast3::ptr_by_sensor(t_sens_data *_sens) { sen = _sens; }

void Fast3::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Fast3::ptr_by_control(t_control *_control) { control = _control; }

void Fast3::ptr_by_map(t_map *_map) { map = _map; }

void Fast3::main_task()
{
    std::cout << "Fast3" << std::endl;
}

void Fast4::ptr_by_sensor(t_sens_data *_sens) { sen = _sens; }

void Fast4::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Fast4::ptr_by_control(t_control *_control) { control = _control; }

void Fast4::ptr_by_map(t_map *_map) { map = _map; }

void Fast4::main_task()
{
    std::cout << "Fast4" << std::endl;
}
