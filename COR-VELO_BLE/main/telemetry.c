#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "string.h"
#include "esp_task_wdt.h"

#define TAG "TELEMETRY"
#define DEVICE_NAME "COR-VELO"
#define SERVICE_UUID 0xFFF0
#define CHAR_UUID_TX 0xFFF1  // Характеристика для передачи данных (устройство → клиент)
#define CHAR_UUID_RX 0xFFF2  // Характеристика для приема команд (клиент → устройство)
#define GATTS_NUM_HANDLE 6
#define TELEMETRY_APP_ID 0
#define MAX_MTU_SIZE 517     // Максимальный поддерживаемый MTU для ESP32
#define QUEUE_SIZE 50  // Вместо 10


static uint8_t telemetry_char_value[512]; // Для TX характеристики
static uint8_t rx_data[512];              // Для RX характеристики
static bool notifications_enabled = false;
static bool device_connected = false;
static QueueHandle_t data_queue = NULL;

static uint32_t packets_sent = 0;
static uint32_t packets_dropped = 0;
// Параметры рекламы для быстрого соединения
// === Глобальные объекты ===
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x100,  // 256*0.625ms = 160ms
    .adv_int_max = 0x200,  
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};
// UUID сервиса
uint8_t service_uuid[16] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, (SERVICE_UUID & 0xFF), (SERVICE_UUID >> 8), 0x00, 0x00
};

// Данные для рекламы

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0,
    .max_interval = 0,
    .appearance = ESP_BLE_APPEARANCE_GENERIC_COMPUTER,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};



static uint16_t telemetry_handle_table[GATTS_NUM_HANDLE];

static esp_gatt_if_t telemetry_gatts_if = 0;
static uint16_t telemetry_conn_id = 0;
static uint16_t current_mtu = 23; // Начальное значение MTU


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void telemetry_notify_task(void *arg);
static void data_generator_task(void *arg);

// GATT атрибуты
// GATT атрибуты
static esp_gatts_attr_db_t telemetry_gatt_db[GATTS_NUM_HANDLE] = {
    // Service Declaration
    [0] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t[]){0x00, 0x28},
            .perm = ESP_GATT_PERM_READ,
            .max_length = sizeof(uint16_t),
            .length = sizeof(uint16_t),
            .value = (uint8_t[]){(SERVICE_UUID & 0xFF), (SERVICE_UUID >> 8)}
        }
    },
    // Characteristic Declaration (TX)
    [1] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t[]){0x03, 0x28},
            .perm = ESP_GATT_PERM_READ,
            .max_length = sizeof(uint8_t),
            .length = sizeof(uint8_t),
            .value = (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_NOTIFY}
        }
    },
    // Characteristic Value (TX)
    [2] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t[]){CHAR_UUID_TX & 0xFF, CHAR_UUID_TX >> 8},
            .perm = ESP_GATT_PERM_READ,
            .max_length = sizeof(telemetry_char_value),
            .length = 0, // Начальная длина 0
            .value = telemetry_char_value // Указатель на буфер
        }
    },
    // Client Characteristic Configuration Descriptor
    [3] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t[]){0x02, 0x29},
            .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            .max_length = sizeof(uint16_t),
            .length = sizeof(uint16_t),
            .value = (uint8_t[]){0x00, 0x00} // Начальное значение 0
        }
    },
    // Characteristic Declaration (RX)
    [4] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t[]){0x03, 0x28},
            .perm = ESP_GATT_PERM_READ,
            .max_length = sizeof(uint8_t),
            .length = sizeof(uint8_t),
            .value = (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR}
        }
    },
    // Characteristic Value (RX)
    [5] = {
        .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = (uint8_t[]){CHAR_UUID_RX & 0xFF, CHAR_UUID_RX >> 8},
            .perm = ESP_GATT_PERM_WRITE,
            .max_length = sizeof(rx_data),
            .length = 0, // Начальная длина 0
            .value = rx_data // Указатель на буфер
        }
    }
};

// Инициализация BLE
void telemetry_init(void) {
    data_queue = xQueueCreate(10, sizeof(uint8_t[MAX_MTU_SIZE]));

    memset(telemetry_char_value, 0, sizeof(telemetry_char_value));
    memset(rx_data, 0, sizeof(rx_data));

    // Настройка MTU
    esp_ble_gatt_set_local_mtu(512);

    // Инициализация BLE
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.controller_task_stack_size = 4096;

    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Регистрируем обратные вызовы
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(TELEMETRY_APP_ID));

    
}


// Запуск задачи отправки данных
void telemetry_start(void) {
    xTaskCreate(telemetry_notify_task, "telemetry_notify_task", 4096, NULL, 5, NULL);
    xTaskCreate(data_generator_task, "data_generator_task", 4096, NULL, 5, NULL);
}



// Генератор тестовых данных (замените на реальные данные с датчиков)
static void data_generator_task(void *arg) {
    uint8_t data_packet[8];
    uint32_t counter = 0;
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
    
    while (1) {
        vTaskDelay(xFrequency);
        
        // Заполнение данных
        int16_t accelX = 1000 + (counter % 100);
        int16_t accelY = 1000 + (counter % 100);
        int16_t accelZ = 1000 + (counter % 100);
        uint16_t angle = counter % 360;
        
        memcpy(&data_packet[0], &accelX, 2);
        memcpy(&data_packet[2], &accelY, 2);
        memcpy(&data_packet[4], &accelZ, 2);
        memcpy(&data_packet[6], &angle, 2);
        
        counter++;
        
        // Отправка в очередь с проверкой подключения
        if (device_connected && xQueueSend(data_queue, data_packet, 0) != pdTRUE) {
            static uint32_t dropped = 0;
            if (++dropped % 100 == 0) {
                ESP_LOGW(TAG, "Packets dropped: %lu", dropped);
            }
        }
    }
}

// Обработчик событий GAP
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Connection parameters updated");
            break;
        default:
            break;
    }
}

// Обработчик событий GATTS
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
    esp_ble_gatts_cb_param_t *param) {
switch (event) {
case ESP_GATTS_REG_EVT:
telemetry_gatts_if = gatts_if;
esp_ble_gap_set_device_name(DEVICE_NAME);
esp_ble_gap_config_adv_data(&adv_data);
esp_ble_gatt_set_local_mtu(MAX_MTU_SIZE);
esp_ble_gatts_create_attr_tab(telemetry_gatt_db, gatts_if, GATTS_NUM_HANDLE, TELEMETRY_APP_ID);
break;

case ESP_GATTS_CREAT_ATTR_TAB_EVT:
memcpy(telemetry_handle_table, param->add_attr_tab.handles, sizeof(telemetry_handle_table));
esp_ble_gatts_start_service(telemetry_handle_table[0]);
break;

case ESP_GATTS_CONNECT_EVT:
device_connected = true;
telemetry_conn_id = param->connect.conn_id;
// Установка быстрых параметров соединения
esp_ble_conn_update_params_t conn_params = {
.min_int = 0x10,    // 10ms
.max_int = 0x20,    // 20ms
.latency = 0,
.timeout = 400,
};
esp_ble_gap_update_conn_params(&conn_params);
break;

case ESP_GATTS_DISCONNECT_EVT:
device_connected = false;
notifications_enabled = false;
esp_ble_gap_start_advertising(&adv_params);
break;

case ESP_GATTS_WRITE_EVT:
if (param->write.handle == telemetry_handle_table[3]) {
// Обработка подписки на уведомления
notifications_enabled = (*(uint16_t *)param->write.value) == 0x0001;
ESP_LOGI(TAG, "Notifications %s", notifications_enabled ? "enabled" : "disabled");
} else if (param->write.handle == telemetry_handle_table[5]) {
// Обработка входящей команды
ESP_LOGI(TAG, "RX Command: %.*s", param->write.len, param->write.value);
}
break;

case ESP_GATTS_MTU_EVT:
ESP_LOGI(TAG, "MTU updated: %d", param->mtu.mtu);
break;

default:
break;
}
}



// Задача отправки данных
static void telemetry_notify_task(void *arg) {
    uint8_t data_to_send[MAX_MTU_SIZE];
    uint32_t counter = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
        
        if (device_connected && notifications_enabled) {
            // Заполняем данные (в реальном приложении здесь будут данные с датчиков)
            for (int i = 0; i < 8; i++) {
                data_to_send[i] = counter++;
            }
            
            esp_err_t ret = esp_ble_gatts_send_indicate(
                telemetry_gatts_if,
                telemetry_conn_id,
                telemetry_handle_table[2], // Handle характеристики TX
                8, // Размер данных
                data_to_send,
                false
            );
            
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(ret));
            }
        }
    }
}