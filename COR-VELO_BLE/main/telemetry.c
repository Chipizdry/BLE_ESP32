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
#define CHAR_UUID_TX 0xFFF1
#define CHAR_UUID_RX 0xFFF2
#define GATTS_NUM_HANDLE 6
#define TELEMETRY_APP_ID 0

static uint8_t rx_data[247];
static bool notifications_enabled = false;
static bool device_connected = false;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x200,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

uint8_t service_uuid[16] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, (SERVICE_UUID & 0xFF), (SERVICE_UUID >> 8), 0x00, 0x00
};

static esp_ble_adv_data_t adv_data = {
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
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static uint16_t telemetry_handle_table[GATTS_NUM_HANDLE];
static uint8_t telemetry_char_value[247];
static esp_gatt_if_t telemetry_gatts_if = 0;
static uint16_t telemetry_conn_id = 0;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void telemetry_notify_task(void *arg);

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

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(TELEMETRY_APP_ID);
}

void telemetry_start(void) {
    xTaskCreate(telemetry_notify_task, "telemetry_notify_task", 4096, NULL, 5, NULL);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&adv_params);
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            telemetry_gatts_if = gatts_if;
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
            esp_ble_gatt_set_local_mtu(247);  // Увеличим MTU
            esp_ble_gatts_create_attr_tab(telemetry_gatt_db, gatts_if, GATTS_NUM_HANDLE, TELEMETRY_APP_ID);
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            memcpy(telemetry_handle_table, param->add_attr_tab.handles, sizeof(telemetry_handle_table));
            esp_ble_gatts_start_service(telemetry_handle_table[0]);
            break;

        case ESP_GATTS_CONNECT_EVT:
            device_connected = true;
            telemetry_conn_id = param->connect.conn_id;
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            device_connected = false;
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == telemetry_handle_table[3]) {
                notifications_enabled = (*(uint16_t *)param->write.value) == 0x0001;
            } else if (param->write.handle == telemetry_handle_table[5]) {
                ESP_LOGI(TAG, "RX Command: %.*s", param->write.len, param->write.value);
                // Можно обрабатывать команды или отвечать
            }
            break;

        default:
            break;
    }
}

static void telemetry_notify_task(void *arg) {
    const TickType_t delay = pdMS_TO_TICKS(10);

    while (1) {
        vTaskDelay(delay);

        if (device_connected && notifications_enabled) {
            uint8_t data_to_send[247];

            // Пример: просто инкремент + шаблон
            static uint8_t counter = 0;
            memset(data_to_send, counter++, sizeof(data_to_send));

            esp_err_t ret = esp_ble_gatts_send_indicate(
                telemetry_gatts_if,
                telemetry_conn_id,
                telemetry_handle_table[2],
                sizeof(data_to_send),
                data_to_send,
                false
            );

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(ret));
            }
        }
    }
}