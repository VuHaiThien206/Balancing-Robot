#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;
    float integ;        /* integral term already multiplied by ki (so ki can change safely) */
    float i_limit;      /* |integral term| limit */
    float out_limit;    /* |output| limit */
} PID_Controller;

void  PID_Init(PID_Controller *p, float kp, float ki, float kd,
               float i_limit, float out_limit);
void  PID_Reset(PID_Controller *p);

/* err  : error (measurement - setpoint)
 * rate : derivative of the error, taken directly from the gyro (deg/s),
 *        so the D term does not amplify noise and has no setpoint kick.
 * dt   : seconds.
 * Output = kp*err + integral + kd*rate, clamped to +-out_limit,
 * with anti-windup (integration stops while saturated in the same direction). */
float PID_Update(PID_Controller *p, float err, float rate, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */