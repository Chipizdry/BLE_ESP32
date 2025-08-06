


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
#define SERVICE_UUID 0x180D 
#define CHAR_UUID    0x2A05
#define GATTS_NUM_HANDLE 4 
#define TELEMETRY_APP_ID 0

// === Глобальные объекты ===
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x100,  // 256*0.625ms = 160ms
    .adv_int_max = 0x200,  
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

// UUID сервиса в виде массива
uint8_t service_uuid[16] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,0x00, 0x10, 0x00, 0x00, (SERVICE_UUID & 0xFF), (SERVICE_UUID >> 8), 0x00, 0x00};
//static uint8_t service_uuid[16] = {0};

// Рекламные данные
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0,  // Должно быть 0 в adv_data
    .max_interval = 0,  // Должно быть 0 в adv_data
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
static uint8_t telemetry_char_value[20];
static esp_gatt_if_t telemetry_gatts_if = 0;
static uint16_t telemetry_conn_id = 0;
static bool device_connected = false;

// === Прототипы ===
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void telemetry_notify_task(void *arg);

// === GATT атрибуты ===
static esp_gatts_attr_db_t telemetry_gatt_db[GATTS_NUM_HANDLE] = {
    // Service Declaration
    [0] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t[]){0x00, 0x28}, // Primary Service UUID
            ESP_GATT_PERM_READ,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){(SERVICE_UUID & 0xFF), (SERVICE_UUID >> 8)}
        }
    },
    // Characteristic Declaration
    [1] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t[]){0x03, 0x28}, // Characteristic Properties
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_NOTIFY}
        }
    },
    // Characteristic Value
    [2] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t[]){(CHAR_UUID & 0xFF), (CHAR_UUID >> 8)},
            ESP_GATT_PERM_READ,
            sizeof(telemetry_char_value),
            telemetry_char_value
        }
    },
    // Client Characteristic Configuration Descriptor (CCCD)
    [3] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t[]){0x02, 0x29}, // CCCD UUID
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0x00} // Default: notifications disabled
        }
    }
};

// === Инициализация BLE ===
void telemetry_init(void) {

    // Аккуратная деинициализация
    esp_err_t ret;
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
        ret = esp_bluedroid_disable();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Bluedroid disable failed: %s", esp_err_to_name(ret));
        }
    }
    
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ret = esp_bluedroid_deinit();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Bluedroid deinit failed: %s", esp_err_to_name(ret));
        }
    }
    
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        ret = esp_bt_controller_disable();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "BT controller disable failed: %s", esp_err_to_name(ret));
        }
    }
    esp_bt_controller_status_t bt_status = esp_bt_controller_get_status();

    if (bt_status == ESP_BT_CONTROLLER_STATUS_IDLE) {
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
        ESP_LOGI(TAG, "BT controller initialized.");
    }

    // Включаем только если ещё не включён
    if (bt_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
        ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
        ESP_LOGI(TAG, "BT controller enabled in BLE mode.");
    }

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ESP_ERROR_CHECK(esp_bluedroid_init());
        ESP_LOGI(TAG, "Bluedroid stack initialized.");
    }

    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        ESP_ERROR_CHECK(esp_bluedroid_enable());
        ESP_LOGI(TAG, "Bluedroid stack enabled.");
    }

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(TELEMETRY_APP_ID));
}


// === Запуск задачи отправки телеметрии ===
void telemetry_start(void) {
    xTaskCreate(telemetry_notify_task, "telemetry_notify_task", 4096, NULL, 5, NULL);
}

// === GAP обработчик ===
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete. Starting advertising...");
            esp_ble_gap_start_advertising(&adv_params);
            break;
        
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started successfully!");
            } else {
                ESP_LOGE(TAG, "Failed to start advertising: %d", param->adv_start_cmpl.status);
            }
            break;
        
        default:
            ESP_LOGI(TAG, "GAP event: %d", event);
            break;
    }
}

// === GATTS обработчик ===
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {

switch (event) {
    case ESP_GATTS_REG_EVT:
    telemetry_gatts_if = gatts_if;
    ESP_LOGI(TAG, "GATTS registered, setting device name");
    
    // 1. Установка имени устройства
    esp_ble_gap_set_device_name(DEVICE_NAME);
    
    // 2. Минимальные рекламные данные
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x100,
        .max_interval = 0x100,
        .appearance = ESP_BLE_APPEARANCE_GENERIC_COMPUTER,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(service_uuid),
        .p_service_uuid = service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
    };
    
    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config adv data failed: %s", esp_err_to_name(ret));
    }
    break;


    case ESP_GATTS_WRITE_EVT:
    if (param->write.handle == telemetry_handle_table[3]) { // Проверяем CCCD handle
        uint16_t cccd_value = *(uint16_t*)param->write.value;
        ESP_LOGI(TAG, "CCCD write: %04x", cccd_value);
        if (cccd_value == 0x0001) {
            ESP_LOGI(TAG, "Notifications enabled");
        } else {
            ESP_LOGI(TAG, "Notifications disabled");
        }
    }
    break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            memcpy(telemetry_handle_table, param->add_attr_tab.handles, sizeof(telemetry_handle_table));
            vTaskDelay(pdMS_TO_TICKS(100)); // Задержка 100мс
            esp_ble_gatts_start_service(telemetry_handle_table[0]);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        device_connected = true;
        telemetry_conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "Client connected.");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        device_connected = false;
        ESP_LOGI(TAG, "Client disconnected. Restarting advertising...");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

// === Задача телеметрии ===
static void telemetry_notify_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(10);  // 500Hz ~= 2мс

    while (1) {
        vTaskDelayUntil(&last_wake_time, frequency);

        if (device_connected) {
            uint8_t telemetry_data[8];
            int16_t accelX = 1000;
            int16_t accelY = 1000;
            int16_t accelZ = 1000;
            uint16_t angle = 360;

            memcpy(&telemetry_data[0], &accelX, 2);
            memcpy(&telemetry_data[2], &accelY, 2);
            memcpy(&telemetry_data[4], &accelZ, 2);
            memcpy(&telemetry_data[6], &angle, 2);

            esp_err_t ret = esp_ble_gatts_send_indicate(
                telemetry_gatts_if,
                telemetry_conn_id,
                telemetry_handle_table[2], // Handle значения характеристики
                sizeof(telemetry_data),
                telemetry_data,
                false
            );
            
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Send indicate failed: %s", esp_err_to_name(ret));
            }
        }
    }
}