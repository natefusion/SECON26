#include "flight_controller.h"

#include <freertos/FreeRTOS.h>
#include <math.h>

#include <Motor/motor.h>
#include <Gyro/bno055.h>

#define DT (0.01f)
#define SLEEP_TIME (DT*1000)

#define MIN_THROTTLE 0.06f
#define ACCEPTABLE_ERROR 0.1f

// arbitrary
#define GYRO_ID 55

#define METER_PER_INCH 0.0254f
#define RAD_PER_DEG 3.141592654f / 180.0f;

struct Pid {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_err;

    float setpoint;
    float integral_limit;
};

enum Pid_Type {
    Pitch, Yaw, Roll, Height, X_Pos, Y_Pos, Num_Pid_Types
};

static float _x_pos_meter = 0.0f;
static float _y_pos_meter = 0.0f;
static float _height_meter = 0.0f;
static float _throttle = 0.0f;
static bool _should_run = true;

static motor_config _cfg[4] = {
    [0] = {.pin = 43, .task_name="M1"},
    [1] = {.pin =  8, .task_name="M2"},
    [2] = {.pin =  1, .task_name="M3"},
    [3] = {.pin =  9, .task_name="M4"},
};

static motor_handler *_mh[4] = {0};

static struct Pid _pid[Num_Pid_Types] = {
    [Pitch] = (struct Pid){
    },
    [Roll] = (struct Pid){
    },
    [Yaw] = (struct Pid){
    },
    [Height] = (struct Pid) {
    },
    [X_Pos] = (struct Pid) {
    },
    [Y_Pos] = (struct Pid) {
    },
};

static inline void clamp(float *x, float y) {
    if (*x < -y) *x = -y;
    if (*x > y) *x = y;
}

static inline void clamp0(float *x, float y) {
    if (*x < 0.0f) *x = 0.0f;
    if (*x > y) *x = y;
}

static float update_pid_angle(struct Pid *p, float angle, float rate) {
    float err = p->setpoint - angle;

    if (_throttle <= MIN_THROTTLE) {
        p->integral = 0.0f;
    } else {
        p->integral += err * DT;
        clamp(&p->integral, p->integral_limit);
    }

    float derivative = rate;

    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp(&output, 1.0f);

    p->prev_err = err;

    return output;
}

static float update_pid_rate(struct Pid *p, float rate) {
    float err = p->setpoint - rate;

    if (_throttle <= MIN_THROTTLE) {
        p->integral = 0.0f;
    } else {
        p->integral += err * DT;
        clamp(&p->integral, p->integral_limit);
    }

    float derivative = (err - p->prev_err) / DT;
    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp(&output, 1.0f);

    p->prev_err = err;

    return output;
}

static void flight_task(void*) {
    while (_should_run) {
        sensors_event_t ev;
        bno055_getEvent1(&ev);
        // ensure everything is in radians pls
        float ax = ev.orientation.x;
        float ay = ev.orientation.y;
        float rx = ev.gyro.x;
        float ry = ev.gyro.y;
        float rz = ev.gyro.z;

        _throttle = update_pid_rate(&_pid[Height], ev.acceleration.z); // is this right?
        clamp0(&_throttle, 0.5f);

        _pid[Pitch].setpoint = update_pid_rate(&_pid[X_Pos], ev.acceleration.x); // is this right?
        clamp(&_pid[Pitch].setpoint, 0.2f);
        
        _pid[Roll].setpoint = update_pid_rate(&_pid[Y_Pos], ev.acceleration.y); // is this right?
        clamp(&_pid[Roll].setpoint, 0.2f);
        
        float p = update_pid_angle(&_pid[Pitch], ax, rx);
        float r = update_pid_angle(&_pid[Roll], ay, ry);
        float y = update_pid_rate(&_pid[Yaw], rz);
        float t = _throttle;

        // ensure y signs are correct
        set_motor_speed_pcnt(_mh[0], t + p + r + y); // front right
        set_motor_speed_pcnt(_mh[1], t + p - r - y); // front left
        set_motor_speed_pcnt(_mh[2], t - p + r + y); // back right
        set_motor_speed_pcnt(_mh[3], t - p - r - y); // back left

        _height_meter += ev.acceleration.z * DT * DT / 2.0f;
        _x_pos_meter += ev.acceleration.x * DT * DT / 2.0f;
        _y_pos_meter += ev.acceleration.y * DT * DT / 2.0f;

        vTaskDelay(SLEEP_TIME / portTICK_PERIOD_MS);    
    }

    set_motor_speed_pcnt(_mh[0], 0); // front right
    set_motor_speed_pcnt(_mh[1], 0); // front left
    set_motor_speed_pcnt(_mh[2], 0); // back right
    set_motor_speed_pcnt(_mh[3], 0); // back left
}

void reset_height(float offset_inches_z) {
    _height_meter = offset_inches_z * METER_PER_INCH;
    _pid[Height].setpoint = _height_meter;
}

void reset_pos(float offset_inches_x, float offset_inches_y) {
    _x_pos_meter = offset_inches_x * METER_PER_INCH;
    _y_pos_meter = offset_inches_y * METER_PER_INCH;
    _pid[X_Pos].setpoint = _x_pos_meter;
    _pid[Y_Pos].setpoint = _y_pos_meter;
}

void change_height_by(float inches_z) {
    _pid[Height].setpoint += inches_z * METER_PER_INCH;
}

void return_to_last_height(void) {
    _pid[Height].setpoint = 0.0f;
}

void change_pos_by(float inches_x, float inches_y) {
    _pid[X_Pos].setpoint += inches_x * METER_PER_INCH;
    _pid[Y_Pos].setpoint += inches_y * METER_PER_INCH;
}

void rotate_by(float degrees) {
    _pid[Yaw].setpoint += degrees * RAD_PER_DEG;
}

bool at_desired_position(void) {
    float e1 = fabsf(_pid[X_Pos].prev_err) < ACCEPTABLE_ERROR;
    float e2 = fabsf(_pid[Y_Pos].prev_err) < ACCEPTABLE_ERROR;
    float e3 = fabsf(_pid[Height].prev_err) < ACCEPTABLE_ERROR;
    float e4 = fabsf(_pid[Yaw].prev_err) < ACCEPTABLE_ERROR;

    return e1 && e2 && e3 && e4;
}

void emergency_stop(void) {
    _should_run = false;
}

bool flight_controller_init(void) {
    bool ok = bno055_begin(GYRO_ID, OPERATION_MODE_NDOF, BNO055_ADDRESS_A);
    if (!ok) {
        printf("Gyro not found");
        return false;
    }

    bno055_setExtCrystalUse(true);

    for (int i = 0; i < 4; ++i) _mh[i] = init_motor(&_cfg[i]);

    xTaskCreate(flight_task, "flight_controller", 4096, NULL, configMAX_PRIORITIES-1, NULL);

    return true;
}
