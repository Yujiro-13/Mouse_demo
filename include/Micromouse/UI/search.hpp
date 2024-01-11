#ifndef SEARCH_HPP
#define SEARCH_HPP

#include <iostream>
#include "UI.hpp"
#include "Micromouse.hpp"

class Search : public UI, Micromouse
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void main_task() override;
    private:
        t_sens_data *sen;   // 後でexternの方を消し、こっちに書き換える
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
};

class All_Search : public UI, Micromouse
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void main_task() override;
    private:
        t_sens_data *sen;   // 後でexternの方を消し、こっちに書き換える
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
};

#endif // SEARCH_HPP