#include "espnow_main.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define MESSAGE_MAGIC UINT32_C(0x534E4F57)

static const char *const TAG = "balance_espnow";
static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {
    0x8c, 0x45, 0x98, 0x8a, 0x6f, 0xd8,
};

// 保持和 espnow_s3 示例相同的报文格式。
typedef struct {
    uint32_t magic;
    uint32_t sequence;
    int32_t value;
} demo_message_t;

_Static_assert(sizeof(demo_message_t) == 12, "Unexpected ESP-NOW message size");

typedef struct {
    uint8_t sender[ESP_NOW_ETH_ALEN];
    demo_message_t message;
} receive_event_t;

static QueueHandle_t receive_queue;
static QueueHandle_t send_status_queue;

static void on_receive(const esp_now_recv_info_t *info, const uint8_t *data, int length)
{
    if (!info || !info->src_addr || !data || length != (int)sizeof(demo_message_t)) {
        return;
    }

    receive_event_t event = {0};
    memcpy(&event.message, data, sizeof(event.message));
    if (event.message.magic != MESSAGE_MAGIC) {
        return;
    }

    // 回调运行在 Wi-Fi 任务中，只复制数据并尝试入队。
    memcpy(event.sender, info->src_addr, sizeof(event.sender));
    (void)xQueueSend(receive_queue, &event, 0);
}

static void on_send(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    (void)xQueueSend(send_status_queue, &status, 0);
}

static void espnow_task(void *arg)
{
    (void)arg;
    demo_message_t message = {
        .magic = MESSAGE_MAGIC,
        .value = CONFIG_DEMO_ESPNOW_VALUE,
    };
    const TickType_t interval_ticks = pdMS_TO_TICKS(CONFIG_DEMO_ESPNOW_INTERVAL_MS);
    const TickType_t send_interval = interval_ticks > 0 ? interval_ticks : 1;
    TickType_t last_attempt = xTaskGetTickCount();
    TickType_t wait_until_send = pdMS_TO_TICKS(100U + esp_random() % 300U);
    bool awaiting_status = false;

    while (true) {
        esp_now_send_status_t status;
        if (awaiting_status && xQueueReceive(send_status_queue, &status, 0) == pdTRUE) {
            awaiting_status = false;
            if (status == ESP_NOW_SEND_SUCCESS) {
                ESP_LOGI(TAG, "TX broadcast | seq=%" PRIu32 " value=%" PRId32,
                         message.sequence, message.value);
            } else {
                ESP_LOGW(TAG, "TX failed | seq=%" PRIu32, message.sequence);
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if (now - last_attempt >= wait_until_send) {
            last_attempt = now;
            wait_until_send = send_interval;
            // 等上一包的回调，迟到的结果仍对应上一包。
            if (!awaiting_status) {
                ++message.sequence;
                esp_err_t result = esp_now_send(BROADCAST_MAC, (const uint8_t *)&message,
                                                sizeof(message));
                awaiting_status = result == ESP_OK;
                if (!awaiting_status) {
                    ESP_LOGW(TAG, "esp_now_send failed: %s", esp_err_to_name(result));
                }
            }
        }

        const TickType_t elapsed = xTaskGetTickCount() - last_attempt;
        const TickType_t wait = elapsed < wait_until_send ? wait_until_send - elapsed : 0;
        receive_event_t event;
        if (xQueueReceive(receive_queue, &event, wait) == pdTRUE) {
            ESP_LOGI(TAG, "RX from " MACSTR " | seq=%" PRIu32 " value=%" PRId32,
                     MAC2STR(event.sender), event.message.sequence, event.message.value);
        }
    }
}

esp_err_t BalanceEspNow_Init(void)
{
    if (receive_queue) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "NVS init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Network init failed");
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    const bool owns_event_loop = ret == ESP_OK;

    receive_queue = xQueueCreate(10, sizeof(receive_event_t));
    send_status_queue = xQueueCreate(1, sizeof(esp_now_send_status_t));
    if (!receive_queue || !send_status_queue) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup_queues;
    }

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_GOTO_ON_ERROR(esp_wifi_init(&wifi_config), cleanup_queues, TAG, "Wi-Fi init failed");
    ESP_GOTO_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), cleanup_wifi, TAG,
                      "Wi-Fi storage setup failed");
    ESP_GOTO_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), cleanup_wifi, TAG, "Wi-Fi mode setup failed");
    ESP_GOTO_ON_ERROR(esp_wifi_start(), cleanup_wifi, TAG, "Wi-Fi start failed");
    ESP_GOTO_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), stop_wifi, TAG, "Wi-Fi power setup failed");
    ESP_GOTO_ON_ERROR(esp_wifi_set_channel(CONFIG_DEMO_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE),
                      stop_wifi, TAG, "Wi-Fi channel setup failed");

    ESP_GOTO_ON_ERROR(esp_now_init(), stop_wifi, TAG, "ESP-NOW init failed");
    ESP_GOTO_ON_ERROR(esp_now_register_recv_cb(on_receive), cleanup_espnow, TAG,
                      "Receive callback setup failed");
    ESP_GOTO_ON_ERROR(esp_now_register_send_cb(on_send), cleanup_espnow, TAG,
                      "Send callback setup failed");

    esp_now_peer_info_t broadcast_peer = {
        .channel = CONFIG_DEMO_ESPNOW_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(broadcast_peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
    ESP_GOTO_ON_ERROR(esp_now_add_peer(&broadcast_peer), cleanup_espnow, TAG,
                      "Broadcast peer setup failed");

    uint8_t own_mac[ESP_NOW_ETH_ALEN];
    ESP_GOTO_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, own_mac), cleanup_espnow, TAG,
                      "Wi-Fi MAC read failed");
    if (xTaskCreate(espnow_task, "balance_espnow", 4096, NULL, 4, NULL) != pdPASS) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup_espnow;
    }

    ESP_LOGI(TAG, "My MAC: " MACSTR " | channel=%d interval=%d ms",
             MAC2STR(own_mac), CONFIG_DEMO_ESPNOW_CHANNEL, CONFIG_DEMO_ESPNOW_INTERVAL_MS);
    return ESP_OK;

cleanup_espnow:
    (void)esp_now_deinit();
stop_wifi:
    (void)esp_wifi_stop();
cleanup_wifi:
    (void)esp_wifi_deinit();
cleanup_queues:
    if (send_status_queue) {
        vQueueDelete(send_status_queue);
        send_status_queue = NULL;
    }
    if (receive_queue) {
        vQueueDelete(receive_queue);
        receive_queue = NULL;
    }
    if (owns_event_loop) {
        (void)esp_event_loop_delete_default();
    }
    return ret;
}
