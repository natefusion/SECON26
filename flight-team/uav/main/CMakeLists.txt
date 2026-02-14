idf_component_register(SRCS
  "uav.c"
  "IR/ir.c"
  "Camera/camera.c"
  "server/server.c"
  "server/commands.c"
  "Gyro/bno055.c"
  "Wifi/wifi.c"
  "Motor/motor.c"
  INCLUDE_DIRS "."
  REQUIRES "esp_timer esp_wifi nvs_flash")
