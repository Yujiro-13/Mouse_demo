#include "include/Micromouse/Motion/motion.hpp"

void Motion::run()
{
    ctl.control_flag = TRUE; // 制御ON
    sens.wall.wall_control = FALSE; // 壁制御OFF

    m_val.max.vel = set_v->max.vel; // 目標（最大）速度設定
    m_val.current.acc = set_m->acc; // 加速度設定
    float end_vel = set_v->end.vel;

    reset_I_gain(); // 積分値リセット
    m_val.current.len = 0.0;

	if (len_count == 7)
	{
		//set_v->tar.len = 45;
		m_val.tar.len = 0.045;
	}else{
		m_val.tar.len = set_v->tar.len;
	}
	
	
    while (((m_val.tar.len - 0.03) - m_val.current.len) > (((m_val.tar.vel)*(m_val.tar.vel) - (end_vel)*(end_vel)) / (2.0 * 
    set_m->acc)))
    {
        vTaskDelay(1);
    }

    //std::cout << "##### deceleration #####" << std::endl;
    m_val.current.acc = -(set_m->acc);

    while ((m_val.tar.len) > m_val.current.len)
    {
        if (m_val.tar.vel <= set_v->max.vel)
        {
            m_val.current.acc = 0;
            m_val.tar.vel = set_v->max.vel;
        }
        vTaskDelay(1);
        
    }
    
    m_val.tar.vel = set_v->tar.vel;
    m_val.current.acc = 0.0;

    std::cout << "run" << std::endl;
}

void Motion::turn()
{
    vTaskDelay(100);
    
    ctl.control_flag = TRUE; // 制御ON
    sens.wall.wall_control = FALSE; // 壁制御OFF
    m_val.current.flag = LEFT; // 左旋回

    m_val.max.ang_vel = set_v->tar.ang_vel; // 目標（最大）角速度設定
    m_val.current.ang_acc = set_m->ang_acc; // 角加速度設定

    m_val.I.vel_error = 0.0;
    m_val.I.ang_error = 0.0;
    reset_I_gain(); // 積分値リセット
    
    //std::cout << "turn_left" << std::endl;
    int turn_count = 0;

    local_rad = m_val.current.rad; // 現在の角度を保存
    
    while(M_PI / 2 > (m_val.current.rad - local_rad)){
        turn_count++;
        //printf("turn_count : %d\n", turn_count);
        vTaskDelay(1);
    }

    //std::cout << "##### deceleration #####" << std::endl;
    ctl.control_flag = FALSE;

    m_val.tar.ang_vel = 0.0;
    m_val.current.ang_acc = 0.0;

    std::cout << "turn" << std::endl;
}

void Motion::stop()
{
    std::cout << "stop" << std::endl;
}

void Motion::back()
{
    std::cout << "back" << std::endl;
}

void Motion::slalom()
{
    std::cout << "slalom" << std::endl;
}

