#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "string.h"


#define TAG "TELEMETRY"
#define DEVICE_NAME "COR-VELO"
#define SERVICE_UUID 0xFFF0
#define CHAR_UUID_TX 0xFFF1  // Характеристика для передачи данных (устройство → клиент)
#define CHAR_UUID_RX 0xFFF2  // Характеристика для приема команд (клиент → устройство)
#define GATTS_NUM_HANDLE 6
#define TELEMETRY_APP_ID 0
#define MAX_MTU_SIZE 517     // Максимальный поддерживаемый MTU для ESP32

//static uint8_t rx_data[247];
static uint8_t rx_data[MAX_MTU_SIZE];
static bool notifications_enabled = false;
static bool device_connected = false;
static QueueHandle_t data_queue = NULL;


// Параметры рекламы для быстрого соединения
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,    // Более короткий интервал рекламы (12.5ms)
    .adv_int_max = 0x40,    // (25ms)
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
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
    .min_interval = 0x20,   // Согласовано с adv_params
    .max_interval = 0x40,   // Согласовано с adv_params
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
static uint8_t telemetry_char_value[MAX_MTU_SIZE];
static esp_gatt_if_t telemetry_gatts_if = 0;
static uint16_t telemetry_conn_id = 0;
static uint16_t current_mtu = 23; // Начальное значение MTU


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void telemetry_notify_task(void *arg);
static void data_generator_task(void *arg);

// GATT атрибуты
static esp_gatts_attr_db_t telemetry_gatt_db[GATTS_NUM_HANDLE] = {
    [0] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x00, 0x28}, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){(SERVICE_UUID & 0xFF), (SERVICE_UUID >> 8)}}},
    [1] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x03, 0x28}, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},
    [2] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){(CHAR_UUID_TX & 0xFF), (CHAR_UUID_TX >> 8)}, ESP_GATT_PERM_READ, sizeof(telemetry_char_value), sizeof(telemetry_char_value), NULL}},
    [3] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x02, 0x29}, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0x00}}},
    [4] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x03, 0x28}, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR}}},
    [5] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){(CHAR_UUID_RX & 0xFF), (CHAR_UUID_RX >> 8)}, ESP_GATT_PERM_WRITE, sizeof(rx_data), sizeof(rx_data), NULL}}
};


void telemetry_init(void) {
    telemetry_gatt_db[2].att_desc.value = telemetry_char_value;
    telemetry_gatt_db[5].att_desc.value = rx_data;

    // Создаем очередь для данных
    data_queue = xQueueCreate(10, sizeof(uint8_t[MAX_MTU_SIZE]));
    if (!data_queue) {
        ESP_LOGE(TAG, "Failed to create data queue");
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(TELEMETRY_APP_ID);
}

void telemetry_start(void) {
    // Запускаем задачи для передачи данных и генерации тестовых данных
    xTaskCreate(telemetry_notify_task, "telemetry_notify_task", 4096, NULL, 5, NULL);
    xTaskCreate(data_generator_task, "data_gen_task", 4096, NULL, 4, NULL);
}


// Генератор тестовых данных (замените на реальные данные с датчиков)
static void data_generator_task(void *arg) {
    uint8_t data_packet[MAX_MTU_SIZE];
    uint32_t counter = 0;
    
    while (1) {
        // Заполняем пакет данными (в реальном приложении здесь будут данные с датчиков)
        for (int i = 0; i < current_mtu; i++) {
            data_packet[i] = (counter + i) & 0xFF;
        }
        counter++;
        
        // Отправляем данные в очередь
        if (xQueueSend(data_queue, data_packet, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGW(TAG, "Data queue full, packet dropped");
        }
        
        // Частота генерации данных (примерно 1kHz)
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Connection params updated: min_int %d, max_int %d, latency %d, timeout %d",
                    param->update_conn_params.min_int,
                    param->update_conn_params.max_int,
                    param->update_conn_params.latency,
                    param->update_conn_params.timeout);
            break;
            
        default:
            break;
    }
}

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

case ESP_GATTS_MTU_EVT:
current_mtu = param->mtu.mtu;
ESP_LOGI(TAG, "MTU updated: %d", current_mtu);
break;

case ESP_GATTS_CREAT_ATTR_TAB_EVT:
memcpy(telemetry_handle_table, param->add_attr_tab.handles, sizeof(telemetry_handle_table));
esp_ble_gatts_start_service(telemetry_handle_table[0]);
break;

case ESP_GATTS_CONNECT_EVT: {
device_connected = true;
telemetry_conn_id = param->connect.conn_id;

// Запрашиваем обновление параметров соединения для более высокой скорости
esp_ble_conn_update_params_t conn_params = {
.min_int = 0x10,    // 10ms
.max_int = 0x20,     // 20ms
.latency = 0,
.timeout = 400,       // 4s
};
esp_ble_gap_update_conn_params(&conn_params);
break;
}

case ESP_GATTS_DISCONNECT_EVT:
device_connected = false;
notifications_enabled = false;
esp_ble_gap_start_advertising(&adv_params);
break;

case ESP_GATTS_WRITE_EVT:
if (param->write.handle == telemetry_handle_table[3]) {
// Включение/выключение уведомлений
notifications_enabled = (*(uint16_t *)param->write.value) == 0x0001;
ESP_LOGI(TAG, "Notifications %s", notifications_enabled ? "enabled" : "disabled");
} else if (param->write.handle == telemetry_handle_table[5]) {
// Обработка входящей команды
ESP_LOGI(TAG, "RX Command: %.*s", param->write.len, param->write.value);

// Пример обработки команды
if (strncmp((char *)param->write.value, "START", 5) == 0) {
notifications_enabled = true;
} else if (strncmp((char *)param->write.value, "STOP", 4) == 0) {
notifications_enabled = false;
}
}
break;

default:
break;
}
}

static void telemetry_notify_task(void *arg) {
    uint8_t data_to_send[MAX_MTU_SIZE];
    
    while (1) {
        if (device_connected && notifications_enabled) {
            // Получаем данные из очереди
            if (xQueueReceive(data_queue, data_to_send, pdMS_TO_TICKS(1)) == pdTRUE) {
                esp_err_t ret = esp_ble_gatts_send_indicate(
                    telemetry_gatts_if,
                    telemetry_conn_id,
                    telemetry_handle_table[2],
                    current_mtu,
                    data_to_send,
                    false
                );

                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(ret));
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}