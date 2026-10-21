#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void motor_init(void);
void motor_left_set(int speed_percent);
void motor_right_set(int speed_percent);
void motor_stop(void);
void motor_test_task(int stacd, void *exinf);
void motor_baseline_task(INT stacd, void *exinf);

#endif /* MOTOR_CONTROL_H */