#include "include/Micromouse/interrupt.hpp"

#define MMPP mot.tire_diameter *M_PI / ENC_MAX
// float _accel = 0.0;

Interrupt::Interrupt() { std::cout << "Interrupt" << std::endl; }

Interrupt::~Interrupt() { std::cout << "~Interrupt" << std::endl; }

void Interrupt::ptr_by_sensor(t_sens_data *_sens) { sen = _sens; }

void Interrupt::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Interrupt::ptr_by_control(t_control *_control) { control = _control; }

void Interrupt::ptr_by_map(t_map *_map) { map = _map; }

void Interrupt::set_module(ADC &_adc, AS5047P &_encR, AS5047P &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot)
{
    adc = &_adc;
    encR = &_encR;
    encL = &_encL;
    buz = &_buz;
    imu = &_imu;
    led = &_led;
    mot = &_mot;
    std::cout << "set_module" << std::endl;
}

void init_structs()
{
    // memset(&s_dir, 0, sizeof(s_dir));
    // memset(&sens.wall, 0, sizeof(sens.wall));
    // memset(&gyro, 0, sizeof(gyro));
    // memset(&enc, 0, sizeof(enc));
    // memset(&motion, 0, sizeof(motion));
    memset(&m_val, 0, sizeof(m_val));
    // memset(&mot, 0, sizeof(mot));
    // memset(&pid, 0, sizeof(pid));
    // memset(&ctl, 0, sizeof(ctl));
    // memset(&wall, 0, sizeof(wall));
    memset(&map, 0, sizeof(map));
    // memset(&mypos, 0, sizeof(mypos));
    // memset(&odom, 0, sizeof(odom));
    ctl.mot.tire_diameter = 0.01368;
    ctl.mot.tire_radius = 0.0066;
    m_val.current.alpha = 0.6;

    // on_logging = xSemaphoreCreateBinary();
}

void Interrupt::calc_target()
{ //  目標値を計算する

    /*std::cout << "m_val.tar.vel : " << m_val.tar.vel << std::endl;
    std::cout << "m_val.max.vel : " << m_val.max.vel << std::endl;
    std::cout << "m_val.current.acc : " << m_val.current.acc << std::endl;
    std::cout << "calc_target" << std::endl;*/

    m_val.tar.vel += (m_val.current.acc) / 1000.0;

    if (m_val.tar.vel > m_val.max.vel)
    {
        m_val.tar.vel = m_val.max.vel;
    }

    // m_val.current.len += m_val.tar.vel;
    // ctl.I.tar.vel += m_val.tar.vel; // 目標積分値更新

    /*std::cout << "m_val.current.len : " << m_val.current.len << std::endl;

    std::cout << "m_val.tar.ang_vel : " << m_val.tar.ang_vel << std::endl;
    std::cout << "m_val.max.ang_vel : " << m_val.max.ang_vel << std::endl;
    std::cout << "m_val.current.ang_acc : " << m_val.current.ang_acc << std::endl;*/

    m_val.tar.ang_vel += (m_val.current.ang_acc) / 1000.0;

    if (m_val.current.flag == LEFT)
    {

        if (m_val.tar.ang_vel > m_val.max.ang_vel)
        {
            m_val.tar.ang_vel = m_val.max.ang_vel;
        }

        // m_val.current.rad += m_val.tar.ang_vel;
    }
    else if (m_val.current.flag == RIGHT)
    {

        if (m_val.tar.ang_vel < m_val.max.ang_vel)
        {
            m_val.tar.ang_vel = m_val.max.ang_vel;
        }
    }

    // ctl.I.tar.ang_vel += m_val.tar.ang_vel; // 目標角速度積分値更新

    // std::cout << "m_val.current.rad : " << m_val.current.rad << std::endl;

    return;
}
void Interrupt::wall_control() //  壁制御
{
    // 左前壁センサ
    if (sens.wall.val.fl > sens.wall.th_wall.fl)
    {
        sens.wall.exist_wall.fl = TRUE;
    }
    else
    {
        sens.wall.exist_wall.fl = FALSE;
    }

    // 右前壁センサ
    if (sens.wall.val.fr > sens.wall.th_wall.fr)
    {
        sens.wall.exist_wall.fr = TRUE;
    }
    else
    {
        sens.wall.exist_wall.fr = FALSE;
    }

    // 左壁センサ
    if (sens.wall.val.l > sens.wall.th_wall.l)
    {
        sens.wall.exist_wall.l = TRUE;
    }
    else
    {
        sens.wall.exist_wall.l = FALSE;
    }
    // 右壁センサ
    if (sens.wall.val.r > sens.wall.th_wall.r)
    {
        sens.wall.exist_wall.r = TRUE;
    }
    else
    {
        sens.wall.exist_wall.r = FALSE;
    }

    if (sens.wall.val.l > sens.wall.th_control.l)
    {
        sens.wall.error.l = sens.wall.val.l - sens.wall.ref.l;
        sens.wall.control_enable.l = TRUE;
    }
    else
    {
        sens.wall.error.l = 0;
        sens.wall.control_enable.l = FALSE;
    }

    if (sens.wall.val.r > sens.wall.th_control.r)
    {
        sens.wall.error.r = sens.wall.val.r - sens.wall.ref.r;
        sens.wall.control_enable.r = TRUE;
    }
    else
    {
        sens.wall.error.r = 0;
        sens.wall.control_enable.r = FALSE;
    }

    if (sens.wall.wall_control == TRUE && sens.wall.val.fl + sens.wall.val.fr <= (sens.wall.th_wall.fl + sens.wall.th_wall.fr) * 5.0)
    {

        if (sens.wall.control_enable.l == TRUE && sens.wall.control_enable.r == TRUE)
        {
            m_val.current.wall_error = sens.wall.error.r - sens.wall.error.l;
        }
        else if (sens.wall.control_enable.l == FALSE && sens.wall.control_enable.r == TRUE)
        {
            m_val.current.wall_error = sens.wall.error.r;
        }
        else if (sens.wall.control_enable.l == TRUE && sens.wall.control_enable.r == FALSE)
        {
            m_val.current.wall_error = -(sens.wall.error.l);
        }
        else
        {
            m_val.current.wall_error = 0;
        }

        m_val.I.wall_error += m_val.current.wall_error / 1000.0;
        m_val.p.wall_error = (m_val.p.wall_error - m_val.current.wall_error) * 1000.0;

        m_val.tar.wall_val = m_val.current.wall_error * (ctl.wall.Kp) + m_val.I.wall_error * (ctl.wall.Ki) - m_val.p.wall_error * (ctl.wall.Kd);
        m_val.tar.ang_vel = m_val.tar.wall_val;

        m_val.p.wall_error = m_val.current.wall_error;
    }
    // xSemaphoreGive(on_logging);

    // std::cout << "wall_ctl" << std::endl;
    return;
}
void Interrupt::feedback_control()
{ // フィードバック制御
    if (ctl.control_flag == TRUE)
    {
        // 速度制御
        m_val.current.vel_error = m_val.tar.vel - m_val.current.vel;
        m_val.I.vel_error += m_val.current.vel_error / 1000.0;
        m_val.p.vel_error = (m_val.p.vel - m_val.current.vel) * 1000.0;

        ctl.V_l = m_val.current.vel_error * (ctl.v.Kp) + m_val.I.vel_error * (ctl.v.Ki) - m_val.p.vel_error * (ctl.v.Kd);
        ctl.V_r = m_val.current.vel_error * (ctl.v.Kp) + m_val.I.vel_error * (ctl.v.Ki) - m_val.p.vel_error * (ctl.v.Kd);

        // 角速度制御
        m_val.current.ang_error = m_val.tar.ang_vel - m_val.current.ang_vel;
        m_val.I.ang_error += m_val.current.ang_error / 1000.0;
        m_val.p.ang_error = (m_val.p.ang_vel - m_val.current.ang_vel) * 1000.0;

        ctl.V_l -= m_val.current.ang_error * (ctl.o.Kp) + m_val.I.ang_error * (ctl.o.Ki) - m_val.p.ang_error * (ctl.o.Kd);
        ctl.V_r += m_val.current.ang_error * (ctl.o.Kp) + m_val.I.ang_error * (ctl.o.Ki) - m_val.p.ang_error * (ctl.o.Kd);

        /*ctl.Duty_l = ctl.V_l / ctl.Vatt;
        ctl.Duty_r = ctl.V_r / ctl.Vatt;*/

        ctl.Duty_l = ctl.V_l / ctl.Vatt;
        ctl.Duty_r = ctl.V_r / ctl.Vatt;

        // std::cout << "ctl.Duty_l : " << ctl.Duty_l << std::endl;
        // std::cout << "ctl.Duty_r : " << ctl.Duty_r << std::endl;

        mot->setMotorSpeed(ctl.Duty_r, ctl.Duty_l, 0.0);
    }
    else
    {
        mot->setMotorSpeed(0.0, 0.0, 0.0);
    }

    // std::cout << "FB_ctl" << std::endl;
    return;
}

void Interrupt::calc_distance()
{ //  走行距離を計算する

    // エンコーダの値を取得
    sens.enc.data.l = -encL->GetData();
    sens.enc.data.r = encR.readAngle();

    sens.enc.locate.l = sens.enc.data.l;
    sens.enc.locate.r = sens.enc.data.r;

    // 差分を計算
    sens.enc.diff_pulse.l = sens.enc.locate.l - sens.enc.p_locate.l;
    sens.enc.diff_pulse.r = sens.enc.locate.r - sens.enc.p_locate.r;

    // 制御１周期分前の値を保持
    sens.enc.p_locate.l = sens.enc.locate.l;
    sens.enc.p_locate.r = sens.enc.locate.r;

    if (sens.enc.diff_pulse.l > ENC_HALF)
        sens.enc.diff_pulse.l -= ENC_MAX - 1;
    if (sens.enc.diff_pulse.l < -ENC_HALF)
        sens.enc.diff_pulse.l += ENC_MAX - 1;
    if (sens.enc.diff_pulse.r > ENC_HALF)
        sens.enc.diff_pulse.r -= ENC_MAX - 1;
    if (sens.enc.diff_pulse.r < -ENC_HALF)
        sens.enc.diff_pulse.r += ENC_MAX - 1;

    float len_L = sens.enc.diff_pulse.l * MMPP;
    float len_R = sens.enc.diff_pulse.r * MMPP;

    // m_val.current.len += (len_L + len_R) / 2.0;

    // std::cout << "m_val.current.len : " << m_val.current.len * 1000.0 << std::endl;

    m_val.l.vel = len_L / 0.001; // 1ms
    m_val.r.vel = len_R / 0.001;

    // std::cout << "m_val.l.vel : " << m_val.l.vel * 1000.0 << std::endl;
    // std::cout << "m_val.r.vel : " << m_val.r.vel *1 000.0 << std::endl;

    if (!imu.in_survaeybias)
    { // サーベイバイアス中は加速度を計算しない
        _accel = (imu.accelY() * 9.80665) * 0.001;
    }

    float _vel = (m_val.l.vel + m_val.r.vel) / 2.0;
    // m_val.current.vel = _vel;
    m_val.current.vel = m_val.current.alpha * (m_val.current.vel + _accel) + (1.0 - m_val.current.alpha) * _vel;
    m_val.current.len += m_val.current.vel * 0.001;
    // std::cout << "m_val.current.vel : " << m_val.current.vel << std::endl;

    m_val.I.vel += m_val.current.vel; // 積分値更新

    m_val.p.vel = m_val.current.vel;

    // std::cout << "calc_dist" << std::endl;
    return;
}

void Interrupt::calc_angle()
{ //  角度を計算する
    // float _yaw = 0.0;
    if (!imu.in_survaeybias)
    { // サーベイバイアス中は角速度を計算しない
        sens.gyro.yaw = imu.gyroZ() - sens.gyro.ref;
    }

    m_val.current.ang_vel = sens.gyro.yaw * (M_PI / 180.0);

    m_val.current.rad += m_val.current.ang_vel / 1000.0;

    m_val.p.ang_vel = m_val.current.ang_vel;

    m_val.I.ang_vel += m_val.current.ang_vel; // 角速度積分値更新

    // std::cout << "m_val.current.rad : " << m_val.current.rad << std::endl;

    // std::cout << "calc_ang" << std::endl;
    // printf("yaw : %f\n", _yaw);
    // printf("ref : %f\n", sens.gyro.ref);
    // printf("imu.in_survaeybias : %d\n", imu.in_survaeybias);
    return;
}

void reset_I_gain()
{
    m_val.I.vel = 0.0;
    m_val.I.ang_vel = 0.0;
    m_val.I.wall_error = 0.0;
    return;
}

void Interrupt::interrupt(void *pvparam)
{ //  xtaskcreate

    while (1)
    {

        calc_target();
        wall_control();
        feedback_control();
        calc_distance();
        calc_angle();

        ctl.time_count++;

        // std::cout << "ctl.time_count : " << ctl.time_count << std::endl;

        vTaskDelay(1);
    }
}
