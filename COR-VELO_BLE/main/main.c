#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "nvs_flash.h"

static const char *TAG = "MAIN";

void telemetry_init(void);
void telemetry_start(void);

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("MAIN", "Free heap: %" PRIu32, esp_get_free_heap_size());

    // 1. Инициализация Bluetooth контроллера
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // 2. Инициализация Bluedroid (BLE стек)
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // 3. Инициализация сервиса телеметрии
    telemetry_init();

    // 4. Запуск задачи уведомлений
    telemetry_start();

    // 5. Оставим задачу "в живых", иначе приложение завершится
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
