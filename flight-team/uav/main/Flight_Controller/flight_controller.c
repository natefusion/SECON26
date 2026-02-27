#include "flight_controller.h"

#include <freertos/FreeRTOS.h>

#include <Motor/motor.h>
#include <Gyro/bno055.h>

#define DT (0.01f)
#define SLEEP_TIME (DT*1000)

#define MIN_THROTTLE 0.06f

struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_err;

    float setpoint;
    float integral_limit;
} Pid;

enum Pid_Type {
    Pitch, Yaw, Roll, Height, Num_Pid_Types
}

static float _throttle = 0.0f;

static motor_config cfg[4] = {
    [0] = {.pin = 43, .task_name="M1"},
    [1] = {.pin =  8, .task_name="M2"},
    [2] = {.pin =  1, .task_name="M3"},
    [3] = {.pin =  9, .task_name="M4"},
};

static motor_handle *mh[4] = {0};

static pid[Num_Pid_Types] = {
    [Pitch] = (struct Pid){
    },
    [Roll] = (struct Pid){
    },
    [Yaw] = (struct Pid){
    },
    [Height] = (struct Pid) {
    },
};

static inline float clamp(float *x, float y) {
    if (x < -y) *x = -y;
    if (x > y) *x = y;
}

static float update_pid_angle(struct Pid *p, float angle, float rate) {
    float err = p->setpoint - angle;

    if (_throttle <= MIN_THROTTLE) {
        p->integral = 0.0f;
    } else {
        p->integral += error * DT;
        clamp(&p->integral, p->integral_limit);
    }

    float derivative = rate;

    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp(&output, p->output_limit);

    return output;
}

static float update_pid_rate(struct Pid *p, float rate) {
    float err = p->setpoint - rate;

    if (_throttle <= MIN_THROTTLE) {
        p->integral = 0.0f;
    } else {
        p->integral += error * DT;
        clamp(&p->integral, p->integral_limit);
    }

    float derivative = (err - p->prev_error) / DT;
    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp(&output, p->output_limit);

    p->prev_err = err;

    return output;
}

void set_pitch(float rad) {
    pid[Pitch].setpoint = rad;
}

void set_roll(float rad) {
    pid[Roll].setpoint = rad;
}

void set_yaw(float rad) {
    pid[Yaw].setpoint = rad;
}

void set_throttle(float t) {
    _throttle = t;
}

static void flight_task(void*) {
    while (true) {
        sensors_event_t ev;
        bno055_getEvent1(&ev);
        // ensure everything is in radians pls
        float ax = ev.orientation.x;
        float ay = ev.orientation.y;
        float az = ev.orientation.z;
        float rx = ev.gyro.x;
        float ry = ev.gyro.y;
        float rz = ev.gyro.z;
        
        float p = update_pid_angle(&p[Pitch], ax, rx);
        float r = update_pid_angle(&p[Roll], ay, ry);
        float y = update_pid_rate(&p[Yaw], az, rz);
        float t = _throttle;

        // ensure y signs are correct
        set_motor_speed_pcnt(mh[0], t + p + r + y); // front right
        set_motor_speed_pcnt(mh[1], t + p - r - y); // front left
        set_motor_speed_pcnt(mh[2], t - p + r + y); // back right
        set_motor_speed_pcnt(mh[3], t - p - r - y); // back left

        vTaskDelay(SLEEP_TIME / portTICK_PERIOD_MS);    
    }
}

bool flight_controller_start(void) {
    book ok = bno055_begin(GYRO_ID, OPERATION_MODE_NDOF, BNO055_ADDRESS_A);
    if (!ok) {
        printf("Gyro not found");
        return false;
    }

    bno055_setExtCrystalUse(true);

    for (int i = 0; i < 4; ++i) mh[i] = init_motor(&cfg[i]);

    xTaskCreate(flight_task, "flight_controller", 4096, NULL, 0, NULL);

    return true;
}
