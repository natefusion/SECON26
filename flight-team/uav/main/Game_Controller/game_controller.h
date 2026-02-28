#include <IR/ir.h>
#include <stdbool.h>

enum Game_State {
    Game_Waiting,
    Game_Launch,
    Game_Send_Codes,
    Game_Retrieve,
};

bool game_state_change_maybe(enum Game_State new_state);
void game_set_ir_code(ir_nec_scan_code_t code);
void game_set_pos_data(float uav_x, float uav_y, float bot_x, float bot_y);
void game_controller_init(void);
