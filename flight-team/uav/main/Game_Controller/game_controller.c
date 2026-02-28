#include "game_controller.h"

#include "freertos/FreeRTOS.h"

#define IR_GPIO 8

static enum Game_State _current_state = Game_Waiting;
static IRtx_t _ir;
static ir_nec_scan_code_t _ir_codes = {0};

static struct {
    float uav_x;
    float uav_y;
    float bot_x;
    float bot_y;
    int prev_frame;
    int frame;
} _pos = {0};


bool game_state_change_maybe(enum Game_State new_state) {
    if (_current_state == Game_Waiting) {
        _current_state = new_state;
        return true;
    } else {
        // ignore request if busy
        return false;
    }
}

void game_set_ir_code(ir_nec_scan_code_t code) {
    _ir_codes = code;
}

void game_set_pos_data(float uav_x, float uav_y, float bot_x, float bot_y) {
    _pos.uav_x = uav_x;
    _pos.uav_y = uav_y;
    _pos.bot_x = bot_x;
    _pos.bot_y = bot_y;
    _pos.prev_frame = _pos.frame;
    _pos.frame += 1;
}

static void launch(void) {
    // set height setpoint
    while (true/*not at setpoint*/) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    // set horizontal setpoint
    while (true/*not at setpoint*/) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void send_codes(void) {
    // set yaw setpoint
    while (true/*not at setpoint*/) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    IRtx_transmit(&_ir, _ir_codes);
}

void retrieve(void) {
    do {
        if (_pos.prev_frame == _pos.frame) continue;
        
        // update horizontal setpoint
        vTaskDelay(20 / portTICK_PERIOD_MS); // what number is good?
    } while (true/*not at setpoint*/);

    // set height setpoint
    while (true/*not at setpoint*/) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void game_task(void*) {
    while (true) {
        switch (_current_state) {
        case Game_Waiting:
            vTaskDelay(500 / portTICK_PERIOD_MS);
            break;
        case Game_Launch:
            launch();
            break;
        case Game_Send_Codes:
            send_codes();
            break;
        case Game_Retrieve:
            retrieve();
            break;
        }

        _current_state = Game_Waiting;
    }
}

void game_controller_init(void) {
    IRtx_init(&_ir, IR_GPIO);
    xTaskCreate(game_task, "game_thread", 4096, NULL, 5, NULL);
}
