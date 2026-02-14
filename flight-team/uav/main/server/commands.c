#include "../Camera/camera.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

static const char *TAG = "TCP_SERVER";

void command_image(int sock) {
    ESP_LOGI(TAG, "Serving IMAGE command...");
    char size_buffer[4];
    memset(size_buffer, 0, 4);
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        ESP_LOGE(TAG, "Error while taking picture");
        int written = send(sock, size_buffer, 4, 0);
        if (written != 4) {
            ESP_LOGE(TAG, "I heard you like errors, so I put a socket error inside of your camera error so you can error while you error");
            return;
        }
    }

    //fill size_buffer with a big-endian encoded version of len
    for (uint32_t i = 0; i < 4; i++) {
        size_buffer[4-i-1] = ((uint32_t)(fb->len)) >> (i*8) & (0xFF);
    }

    ESP_LOGI(TAG, "Image length: %d", fb->len);

    int written = send(sock, size_buffer, 4, 0);
    if (written != 4) {
        ESP_LOGE(TAG, "Error occurred while sending image size: errno %d", errno);
        esp_camera_fb_return(fb);
        return;
    }

    written = send(sock, fb->buf, fb->len, 0);
    if (written != fb->len) {
        ESP_LOGE(TAG, "Error occurred while sending image data: errno %d", errno);
        esp_camera_fb_return(fb);
        return;
    }
    esp_camera_fb_return(fb);
};

void command_launch(int sock) {

}

void command_retrieve(int sock) {

}

void command_transmission_codes(int sock) {

}

void command_pos(int sock) {

}

void command_stop(int sock) {

}
