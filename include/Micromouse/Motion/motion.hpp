#ifndef MOTION_HPP
#define MOTION_HPP

#include <iostream>
#include "structs.hpp"
#include "Micromouse.hpp"

class Motion : public Micromouse
{
    public:
        void ptr_by_sensor(t_sens_data *sens) override;
        void ptr_by_motion(t_mouse_motion_val *val) override;
        void ptr_by_control(t_control *control) override;
        void ptr_by_map(t_map *map) override;
        void run();
        void turn();
        void stop();
        void back();
        void slalom();
    protected:
        t_sens_data *sen;   // 後でexternの方を消し、こっちに書き換える。ここは、privateかprotected要検討
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
    private:
};

#endif // MOTION_HPP