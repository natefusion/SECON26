#include "game_controller.h"

#include "freertos/FreeRTOS.h"

static enum Game_State current_state = Game_Waiting;

bool game_state_change_maybe(enum Game_State new_state) {
    if (current_state == Game_Waiting) {
        current_state = new_state;
        return true;
    } else {
        // ignore request if busy
        return false;
    }
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

    // send ir codes (where are they???)
}

void retrieve(void) {
    do {
        // get coords
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
        switch (current_state) {
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

        current_state = Game_Waiting;
    }
}

void game_start(void) {
    xTaskCreate(game_task, "game_thread", 4096, NULL, 5, NULL);
}
