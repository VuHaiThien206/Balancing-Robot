#include "pid.h"

void PID_Init(PID_Controller *p, float kp, float ki, float kd,
              float i_limit, float out_limit)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_limit = i_limit;
    p->out_limit = out_limit;
    p->integ = 0.0f;
}

void PID_Reset(PID_Controller *p)
{
    p->integ = 0.0f;
}

float PID_Update(PID_Controller *p, float err, float rate, float dt)
{
    float i_new = p->integ + p->ki * err * dt;
    if (i_new >  p->i_limit) i_new =  p->i_limit;
    if (i_new < -p->i_limit) i_new = -p->i_limit;

    float u = p->kp * err + i_new + p->kd * rate;

    /* anti-windup: do not keep integrating into saturation */
    if (u > p->out_limit) {
        u = p->out_limit;
        if (err > 0.0f) i_new = p->integ;
    } else if (u < -p->out_limit) {
        u = -p->out_limit;
        if (err < 0.0f) i_new = p->integ;
    }

    p->integ = i_new;
    return u;
}