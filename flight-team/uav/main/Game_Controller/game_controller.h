#include <stdbool.h>

enum Game_State {
    Game_Waiting,
    Game_Launch,
    Game_Send_Codes,
    Game_Retrieve,
    Game_STOP,
};

bool game_state_change_maybe(enum Game_State new_state);
void game_start(void);
