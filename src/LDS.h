
#ifndef LDS_H
#define LDS_H

/**
  *
  *
  * 
  */
struct _LDS_PID
{
    float Setpoint;        // set speed value
    float ActualSpeed;     // actual speed
//    float error;            // set_speed - actual_speed
    float PreError;        // error from last time (previous Error)
    float Kp;           // (P)roportional Tuning Parameter
    float Ki;           // (I)ntegral Tuning Parameter
    float Kd;           // (D)erivative Tuning Parameter
//    float Dt;           // delta time
    float Integral;         // 
    float Derivative;
};

void UART5_RX_SetEmpty(void);

void LDS_PID_Init(float Kp, float Ki, float Kd);
float LDS_PID_Update(float speed);
void LDS_PWM_Update(float duty);

int32_t lds_get_packet(void);
void lds(void);

#endif /* LDS_H */
/* end of LDS.h */
