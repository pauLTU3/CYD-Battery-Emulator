// pauLTU3
// ESP-NOW receiver and UI updates.
// Testing build, not validated on a real system.

#include "espnow_receiver.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <lvgl.h>

#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#include "screen_network.h"
#include "ui.h"

namespace {

// Packet ids must stay aligned with the Battery Emulator sender.
enum espnow_message_enum : uint8_t {
    BAT_INFO = 1,
    BAT_STATUS = 2,
    BAT_BALANCE = 3,
    BAT_CELL_STATUS = 4,
    BAT_CELL_STATUS_16BIT = 5,
    BAT_FAULT_TEXT = 6,
    BAT_REMOTE_COMMAND = 7,
    BAT_REMOTE_STATE = 8,
    BAT_FAULT_EVENTS = 9,
    BAT_AUX_INFO = 10,
    BAT_REMOTE_NET_INFO = 11
};

enum espnow_remote_command_id : uint8_t {
    REMOTE_CMD_SET_PAUSE = 1,
    REMOTE_CMD_SET_CONTACTORS = 2,
    REMOTE_CMD_REBOOT = 3
};

enum espnow_remote_command_result : uint8_t {
    REMOTE_CMD_RESULT_NONE = 0,
    REMOTE_CMD_RESULT_OK = 1,
    REMOTE_CMD_RESULT_INVALID = 2
};

#define GENERATE_STRING(STRING) #STRING,
#define RECEIVER_EVENTS_ENUM_TYPE(XX)       \
    XX(EVENT_CANMCP2518FD_INIT_FAILURE)     \
    XX(EVENT_CANMCP2515_INIT_FAILURE)       \
    XX(EVENT_CANFD_BUFFER_FULL)             \
    XX(EVENT_CAN_BUFFER_FULL)               \
    XX(EVENT_CAN_CORRUPTED_WARNING)         \
    XX(EVENT_CAN_BATTERY_MISSING)           \
    XX(EVENT_CAN_BATTERY2_MISSING)          \
    XX(EVENT_CAN_BATTERY3_MISSING)          \
    XX(EVENT_CAN_CHARGER_MISSING)           \
    XX(EVENT_CAN_INVERTER_MISSING)          \
    XX(EVENT_CAN_NATIVE_TX_FAILURE)         \
    XX(EVENT_CHARGE_LIMIT_EXCEEDED)         \
    XX(EVENT_CONTACTOR_WELDED)              \
    XX(EVENT_CONTACTOR_OPEN)                \
    XX(EVENT_DISCHARGE_LIMIT_EXCEEDED)      \
    XX(EVENT_WATER_INGRESS)                 \
    XX(EVENT_12V_LOW)                       \
    XX(EVENT_SOC_PLAUSIBILITY_ERROR)        \
    XX(EVENT_SOC_UNAVAILABLE)               \
    XX(EVENT_STALE_VALUE)                   \
    XX(EVENT_KWH_PLAUSIBILITY_ERROR)        \
    XX(EVENT_BALANCING_START)               \
    XX(EVENT_BALANCING_END)                 \
    XX(EVENT_BATTERY_EMPTY)                 \
    XX(EVENT_BATTERY_FULL)                  \
    XX(EVENT_BATTERY_FUSE)                  \
    XX(EVENT_BATTERY_FROZEN)                \
    XX(EVENT_BATTERY_CAUTION)               \
    XX(EVENT_BATTERY_CHG_STOP_REQ)          \
    XX(EVENT_BATTERY_DISCHG_STOP_REQ)       \
    XX(EVENT_BATTERY_CHG_DISCHG_STOP_REQ)   \
    XX(EVENT_BATTERY_OVERHEAT)              \
    XX(EVENT_BATTERY_OVERVOLTAGE)           \
    XX(EVENT_BATTERY_UNDERVOLTAGE)          \
    XX(EVENT_BATTERY_VALUE_UNAVAILABLE)     \
    XX(EVENT_BATTERY_ISOLATION)             \
    XX(EVENT_BATTERY_REQUESTS_HEAT)         \
    XX(EVENT_BATTERY_WARMED_UP)             \
    XX(EVENT_BATTERY_SOC_RECALIBRATION)     \
    XX(EVENT_BYD_AUTO_SOC_CALIBRATION)      \
    XX(EVENT_BATTERY_SOC_RESET_SUCCESS)     \
    XX(EVENT_BATTERY_SOC_RESET_FAIL)        \
    XX(EVENT_VOLTAGE_DIFFERENCE)            \
    XX(EVENT_SOH_DIFFERENCE)                \
    XX(EVENT_SOH_LOW)                       \
    XX(EVENT_HVIL_FAILURE)                  \
    XX(EVENT_LOW_HEAP_MEMORY)               \
    XX(EVENT_PRECHARGE_FAILURE)             \
    XX(EVENT_INTERNAL_OPEN_FAULT)           \
    XX(EVENT_INVERTER_OPEN_CONTACTOR)       \
    XX(EVENT_INTERFACE_MISSING)             \
    XX(EVENT_MODBUS_INVERTER_MISSING)       \
    XX(EVENT_NO_ENABLE_DETECTED)            \
    XX(EVENT_ERROR_OPEN_CONTACTOR)          \
    XX(EVENT_CELL_CRITICAL_UNDER_VOLTAGE)   \
    XX(EVENT_CELL_CRITICAL_OVER_VOLTAGE)    \
    XX(EVENT_CELL_UNDER_VOLTAGE)            \
    XX(EVENT_CELL_OVER_VOLTAGE)             \
    XX(EVENT_CELL_DEVIATION_HIGH)           \
    XX(EVENT_UNKNOWN_EVENT_SET)             \
    XX(EVENT_OTA_UPDATE)                    \
    XX(EVENT_OTA_UPDATE_TIMEOUT)            \
    XX(EVENT_DUMMY_INFO)                    \
    XX(EVENT_DUMMY_DEBUG)                   \
    XX(EVENT_DUMMY_WARNING)                 \
    XX(EVENT_DUMMY_ERROR)                   \
    XX(EVENT_PERSISTENT_SAVE_INFO)          \
    XX(EVENT_SERIAL_RX_WARNING)             \
    XX(EVENT_SERIAL_RX_FAILURE)             \
    XX(EVENT_SERIAL_TX_FAILURE)             \
    XX(EVENT_SERIAL_TRANSMITTER_FAILURE)    \
    XX(EVENT_SMA_PAIRING)                   \
    XX(EVENT_TASK_OVERRUN)                  \
    XX(EVENT_THERMAL_RUNAWAY)               \
    XX(EVENT_RECOVERY_START)                \
    XX(EVENT_RECOVERY_END)                  \
    XX(EVENT_RESET_UNKNOWN)                 \
    XX(EVENT_RESET_POWERON)                 \
    XX(EVENT_RESET_EXT)                     \
    XX(EVENT_RESET_SW)                      \
    XX(EVENT_RESET_PANIC)                   \
    XX(EVENT_RESET_INT_WDT)                 \
    XX(EVENT_RESET_TASK_WDT)                \
    XX(EVENT_RESET_WDT)                     \
    XX(EVENT_RESET_DEEPSLEEP)               \
    XX(EVENT_RESET_BROWNOUT)                \
    XX(EVENT_RESET_SDIO)                    \
    XX(EVENT_RESET_USB)                     \
    XX(EVENT_RESET_JTAG)                    \
    XX(EVENT_RESET_EFUSE)                   \
    XX(EVENT_RESET_PWR_GLITCH)              \
    XX(EVENT_RESET_CPU_LOCKUP)              \
    XX(EVENT_RJXZS_LOG)                     \
    XX(EVENT_PAUSE_BEGIN)                   \
    XX(EVENT_PAUSE_END)                     \
    XX(EVENT_PID_FAILED)                    \
    XX(EVENT_WIFI_CONNECT)                  \
    XX(EVENT_WIFI_DISCONNECT)               \
    XX(EVENT_MQTT_CONNECT)                  \
    XX(EVENT_MQTT_DISCONNECT)               \
    XX(EVENT_EQUIPMENT_STOP)                \
    XX(EVENT_AUTOMATIC_PRECHARGE_FAILURE)   \
    XX(EVENT_SD_INIT_FAILED)                \
    XX(EVENT_PERIODIC_BMS_RESET)            \
    XX(EVENT_PERIODIC_BMS_RESET_FAILURE)    \
    XX(EVENT_BMS_RESET_REQ_SUCCESS)         \
    XX(EVENT_BMS_RESET_REQ_FAIL)            \
    XX(EVENT_BATTERY_TEMP_DEVIATION_HIGH)   \
    XX(EVENT_GPIO_NOT_DEFINED)              \
    XX(EVENT_GPIO_CONFLICT)                 \
    XX(EVENT_NOF_EVENTS)

// Event order must match the Battery Emulator sender.
static const char *kEventNames[] = {RECEIVER_EVENTS_ENUM_TYPE(GENERATE_STRING)};

static const char *kEventLevels[] = {"INFO", "DEBUG", "WARNING", "ERROR", "UPDATE"};

enum real_bms_status_enum {
    BMS_DISCONNECTED = 0,
    BMS_STANDBY = 1,
    BMS_ACTIVE = 2,
    BMS_FAULT = 3
};

enum system_status_enum {
    SYS_STANDBY = 0,
    SYS_INACTIVE = 1,
    SYS_DARKSTART = 2,
    SYS_ACTIVE = 3,
    SYS_FAULT = 4,
    SYS_UPDATING = 5
};

struct BATTERY_INFO_TYPE {
    uint32_t total_capacity_Wh = 30000;
    uint32_t reported_total_capacity_Wh = 30000;
    uint16_t max_design_voltage_dV = 5000;
    uint16_t min_design_voltage_dV = 2500;
    uint16_t max_cell_voltage_mV = 4300;
    uint16_t min_cell_voltage_mV = 2700;
    uint16_t max_cell_voltage_deviation_mV = 500;
    uint8_t number_of_cells = 0;
    int chemistry = 1;
};

struct BATTERY_STATUS_TYPE {
    uint32_t remaining_capacity_Wh = 0;
    uint32_t reported_remaining_capacity_Wh = 0;
    uint32_t max_discharge_power_W = 0;
    uint32_t max_charge_power_W = 0;
    uint32_t override_discharge_power_W = 0;
    uint32_t override_charge_power_W = 0;
    int32_t active_power_W = 0;
    int32_t total_charged_battery_Wh = 0;
    int32_t total_discharged_battery_Wh = 0;
    uint16_t max_discharge_current_dA = 0;
    uint16_t max_charge_current_dA = 0;
    uint16_t soh_pptt = 0;
    uint16_t voltage_dV = 0;
    uint16_t cell_max_voltage_mV = 0;
    uint16_t cell_min_voltage_mV = 0;
    uint16_t real_soc = 0;
    uint16_t reported_soc = 0;
    uint16_t CAN_error_counter = 0;
    int16_t temperature_max_dC = 0;
    int16_t temperature_min_dC = 0;
    int16_t current_dA = 0;
    int16_t reported_current_dA = 0;
    uint8_t CAN_battery_still_alive = 0;
    int real_bms_status = BMS_DISCONNECTED;
    int led_mode = 0;
    int balancing_status = 0;
};

// 8-bit cell data in 20 mV steps.
struct BATTERY_CELL_STATUS_TYPE {
    uint8_t cell_voltages_mV[192] = {0};
    uint8_t number_of_cells = 0;
};

// Chunked full-resolution cell data.
struct BATTERY_CELL_STATUS_16BIT_CHUNK_TYPE {
    uint8_t transfer_id;
    uint8_t total_cells;
    uint8_t start_index;
    uint8_t cells_in_chunk;
    uint16_t cell_voltages_mV[]; 
};

// Chunked fault text.
struct BATTERY_FAULT_TEXT_CHUNK_TYPE {
    uint8_t transfer_id;
    uint8_t total_chunks;
    uint8_t chunk_index;
    uint8_t text_length;
    char text[];
};

// One structured fault/event record.
struct BATTERY_FAULT_EVENT_RECORD_TYPE {
    uint8_t event_id;
    uint8_t level;
    uint8_t data;
    uint8_t count;
    uint32_t age_seconds;
} __attribute__((packed));

// Chunked transport for structured fault records.
struct BATTERY_FAULT_EVENTS_CHUNK_TYPE {
    uint8_t transfer_id;
    uint8_t total_events;
    uint8_t start_index;
    uint8_t records_in_chunk;
    BATTERY_FAULT_EVENT_RECORD_TYPE records[];
} __attribute__((packed));

// Command packet sent from CYD to Battery Emulator.
struct BATTERY_REMOTE_COMMAND_TYPE {
    uint8_t command_id;
    uint8_t command_sequence;
    uint8_t command_value;
    uint8_t reserved;
} __attribute__((packed));

// Extra battery data that does not fit nicely into the main status packet.
struct BATTERY_AUX_INFO_TYPE {
    uint32_t isolation_resistance_kohm = 0;
} __attribute__((packed));

// Optional network info packet from the emulator side.
struct BATTERY_REMOTE_NET_INFO_TYPE {
    uint8_t sta_connected = 0;
    int16_t rssi_dbm = 0;
    char ip_address[16] = {0};
    char ssid[32] = {0};
} __attribute__((packed));

// Remote state packet used for system tint and status telemetry.
struct BATTERY_REMOTE_STATE_TYPE {
    uint8_t ack_sequence;
    uint8_t ack_command_id;
    uint8_t ack_result;
    uint8_t pause_status;
    uint8_t pause_request_on;
    uint8_t equipment_stop_active;
    uint8_t system_status;
    uint8_t contactors_engaged;
    uint8_t inverter_allows_contactor_closing;
    uint8_t battery1_contactors_engaged;
    uint8_t battery2_contactors_engaged;
} __attribute__((packed));

// Common 4-byte ESP-NOW header present on every packet.
struct ESPNOW_HEADER {
    uint16_t emulator_id;
    uint8_t battery_id;
    uint8_t esp_message_type;
} __attribute__((packed));

// Shared state per battery slot, written by the ESP-NOW callback.
struct SharedBatteryState {
    bool has_info = false;
    bool has_status = false;
    bool has_cells = false;
    bool has_cells_16bit = false;
    bool has_fault_text = false;
    bool has_fault_events = false;
    bool has_aux_info = false;
    bool has_remote_net_info = false;
    bool has_remote_state = false;
    bool dirty = false;
    bool rx_event_pending = false;
    uint32_t last_packet_ms = 0;
    uint32_t last_info_ms = 0;
    uint32_t last_status_ms = 0;
    uint32_t last_cells_ms = 0;
    uint32_t last_aux_info_ms = 0;
    uint16_t emulator_id = 0;
    uint8_t battery_id = 0;
    uint8_t last_message_type = 0;
    int last_message_len = 0;
    uint32_t rx_packet_count = 0;
    uint32_t ignored_packet_count = 0;
    BATTERY_INFO_TYPE info;
    BATTERY_STATUS_TYPE status;
    BATTERY_AUX_INFO_TYPE aux_info;
    BATTERY_REMOTE_NET_INFO_TYPE remote_net_info;
    BATTERY_CELL_STATUS_TYPE cells;
    uint16_t cells_16bit_mV[192] = {0};
    uint32_t last_cells_16bit_ms = 0;
    uint8_t cell16_transfer_id = 0;
    uint8_t cell16_expected_chunks = 0;
    uint8_t cell16_received_chunks_mask = 0;
    uint8_t cell16_total_cells = 0;
    char fault_text[1536] = {0};
    uint32_t last_fault_text_ms = 0;
    uint8_t fault_transfer_id = 0;
    uint8_t fault_expected_chunks = 0;
    uint32_t fault_received_chunks_mask = 0;
    BATTERY_FAULT_EVENT_RECORD_TYPE fault_events[128] = {};
    uint8_t fault_event_count = 0;
    uint32_t last_fault_events_ms = 0;
    uint8_t fault_events_transfer_id = 0;
    BATTERY_REMOTE_STATE_TYPE remote_state{};
    uint32_t last_remote_state_ms = 0;
    uint8_t last_sent_command_sequence = 0;
    uint8_t pending_command_sequence = 0;
    uint8_t pending_command_id = 0;
    uint8_t pending_command_value = 0;
    uint32_t pending_command_sent_ms = 0;
    bool command_pending = false;
};

// Local snapshot used for LVGL updates outside the critical section.
struct UiBatteryState {
    bool has_info = false;
    bool has_status = false;
    bool has_cells = false;
    bool has_cells_16bit = false;
    bool has_fault_text = false;
    bool has_fault_events = false;
    bool has_aux_info = false;
    bool has_remote_net_info = false;
    bool has_remote_state = false;
    uint32_t last_packet_ms = 0;
    uint32_t last_info_ms = 0;
    uint32_t last_status_ms = 0;
    uint32_t last_cells_ms = 0;
    uint32_t last_aux_info_ms = 0;
    uint16_t emulator_id = 0;
    uint8_t battery_id = 0;
    uint8_t last_message_type = 0;
    int last_message_len = 0;
    uint32_t rx_packet_count = 0;
    uint32_t ignored_packet_count = 0;
    bool rx_event_pending = false;
    BATTERY_INFO_TYPE info;
    BATTERY_STATUS_TYPE status;
    BATTERY_AUX_INFO_TYPE aux_info;
    BATTERY_REMOTE_NET_INFO_TYPE remote_net_info;
    BATTERY_CELL_STATUS_TYPE cells;
    uint16_t cells_16bit_mV[192] = {0};
    uint32_t last_cells_16bit_ms = 0;
    uint8_t cell16_total_cells = 0;
    char fault_text[1536] = {0};
    uint32_t last_fault_text_ms = 0;
    BATTERY_FAULT_EVENT_RECORD_TYPE fault_events[128] = {};
    uint8_t fault_event_count = 0;
    uint32_t last_fault_events_ms = 0;
    BATTERY_REMOTE_STATE_TYPE remote_state{};
    uint32_t last_remote_state_ms = 0;
    uint8_t last_sent_command_sequence = 0;
    uint8_t pending_command_sequence = 0;
    uint8_t pending_command_id = 0;
    uint8_t pending_command_value = 0;
    uint32_t pending_command_sent_ms = 0;
    bool command_pending = false;
};

// Smaller view used when only remote-state data is needed.
struct UiRemoteStateView {
    bool has_remote_state = false;
    uint32_t last_remote_state_ms = 0;
    BATTERY_REMOTE_STATE_TYPE remote_state{};
    uint8_t pending_command_id = 0;
    uint32_t pending_command_sent_ms = 0;
    bool command_pending = false;
};

// Default receiver channel when STA Wi-Fi is not active.
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;

// Broadcast peer used for command send.
uint8_t g_broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Packet freshness and UI timing.
constexpr uint32_t PACKET_TIMEOUT_MS = 10000;
constexpr uint32_t HIGH_RES_CELL_TIMEOUT_MS = 10000;
// BE v2 replays the event batch every 10 s; keep the last message through one
// scheduling interval so the Faults page does not flicker between batches.
constexpr uint32_t FAULT_TEXT_TIMEOUT_MS = 15000;
constexpr uint32_t FAULT_EVENTS_TIMEOUT_MS = 15000;
constexpr uint32_t REMOTE_STATE_TIMEOUT_MS = 10000;
constexpr uint32_t COMMAND_ACK_TIMEOUT_MS = 2000;
constexpr uint32_t UI_STALE_HOLD_MS = 10000;
constexpr uint16_t MAX_UI_CHART_POINTS = 192;
constexpr uint8_t BATTERY_SLOT_COUNT = 2;
constexpr size_t ESPNOW_HEADER_SIZE = sizeof(ESPNOW_HEADER);
// Battery Emulator telemetry protocol v2 (TLV wire format).
constexpr uint8_t ESPNOW_V2_MAGIC_0 = 0x42;
constexpr uint8_t ESPNOW_V2_MAGIC_1 = 0x45;
constexpr uint8_t ESPNOW_V2_VERSION = 2;
constexpr uint8_t ESPNOW_V2_FLAG_MORE_CHUNKS = 0x01;
constexpr size_t ESPNOW_V2_HEADER_SIZE = 12;
enum : uint8_t {
    V2_FRAME_SYSTEM = 0x01, V2_FRAME_BATTERY = 0x02,
    V2_FRAME_CELLS = 0x03, V2_FRAME_EVENT = 0x04
};
enum : uint8_t {
    V2_FW_VERSION = 0x01, V2_HOSTNAME = 0x02, V2_SYSTEM_STATUS = 0x04,
    V2_PAUSE_STATUS = 0x05, V2_EMULATOR_STATUS = 0x07, V2_BATTERY_COUNT = 0x0A,
    V2_WIFI_RSSI = 0x0B, V2_INVERTER_ALIVE = 0x0C, V2_CONTACTORS = 0x0D, V2_EQUIPMENT_STOP = 0x0F,
    V2_IP_ADDRESS = 0x10, V2_SSID = 0x11,
    V2_NUMBER_OF_CELLS = 0x30, V2_CHEMISTRY = 0x31, V2_TOTAL_CAPACITY = 0x32,
    V2_REPORTED_CAPACITY = 0x33, V2_MAX_DESIGN_VOLTAGE = 0x34, V2_MIN_DESIGN_VOLTAGE = 0x35,
    V2_MAX_CELL_DESIGN = 0x36, V2_MIN_CELL_DESIGN = 0x37, V2_MAX_CELL_DEVIATION = 0x38,
    V2_SOC = 0x50, V2_SOC_REAL = 0x51, V2_SOH = 0x52, V2_VOLTAGE = 0x53,
    V2_CURRENT = 0x54, V2_REPORTED_CURRENT = 0x55, V2_ACTIVE_POWER = 0x56,
    V2_REMAINING = 0x57, V2_REPORTED_REMAINING = 0x58, V2_MAX_CHARGE_POWER = 0x59,
    V2_MAX_DISCHARGE_POWER = 0x5A, V2_MAX_CHARGE_CURRENT = 0x5B,
    V2_MAX_DISCHARGE_CURRENT = 0x5C, V2_OVERRIDE_CHARGE = 0x5D, V2_OVERRIDE_DISCHARGE = 0x5E,
    V2_CELL_MAX = 0x5F, V2_CELL_MIN = 0x60, V2_TEMP_MAX = 0x61, V2_TEMP_MIN = 0x62,
    V2_TOTAL_CHARGED = 0x63, V2_TOTAL_DISCHARGED = 0x64, V2_INSULATION = 0x65,
    V2_BALANCING = 0x66, V2_REAL_BMS = 0x6A, V2_CAN_ALIVE = 0x6B,
    V2_CAN_ERRORS = 0x6C, V2_LED_MODE = 0x6D, V2_DETECTED = 0x6E,
    V2_CELL_COUNT = 0x90, V2_CELL_INDEX = 0x91, V2_CELL_VOLTAGES = 0x92,
    V2_CELL_BALANCING = 0x93, V2_EVENT_ID = 0xA0, V2_EVENT_NAME = 0xA1,
    V2_EVENT_SEVERITY = 0xA2, V2_EVENT_STATE = 0xA3, V2_EVENT_COUNT = 0xA4,
    V2_EVENT_MILLIS = 0xA6, V2_EVENT_MESSAGE = 0xA7, V2_EVENT_INDEX = 0xA8,
    V2_EVENT_TOTAL = 0xA9, V2_EVENT_DATA_I16 = 0xAA
};
constexpr uint32_t RX_LOG_INTERVAL_MS = 1000;
constexpr uint8_t RED_LED_PIN = 4;
constexpr uint8_t GREEN_LED_PIN = 16;
constexpr uint8_t BLUE_LED_PIN = 17;

portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

SharedBatteryState g_shared_states[BATTERY_SLOT_COUNT];

lv_chart_series_t *g_chart_series[BATTERY_SLOT_COUNT] = {nullptr, nullptr};

lv_coord_t g_cell_chart_values[BATTERY_SLOT_COUNT][MAX_UI_CHART_POINTS] = {};

uint32_t g_last_rx_log_ms = 0;
uint32_t g_last_ui_log_ms = 0;
bool g_waiting_log_printed = false;
int g_last_led_status = -999;
bool g_ui_showing_waiting[BATTERY_SLOT_COUNT] = {false, false};
int g_last_screen_tint = -999;
lv_obj_t *g_main_ip_label = nullptr;
lv_obj_t *g_main_ip_parent = nullptr;

// Convert Battery Emulator battery_id into local slot index.
int battery_slot_from_id(uint8_t battery_id) {
    if (battery_id >= 1U && battery_id <= BATTERY_SLOT_COUNT) {
        return static_cast<int>(battery_id - 1U);
    }
    return -1;
}

// Slot 0 is treated as the primary battery for remote-control state.
SharedBatteryState &primary_shared_state() {
    return g_shared_states[0];
}

// Translate battery status code to user-visible text.
const char *status_to_text(int status) {
    switch (status) {
        case BMS_STANDBY:
            return "STANDBY";
        case BMS_ACTIVE:
            return "ACTIVE";
        case BMS_FAULT:
            return "FAULT";
        case BMS_DISCONNECTED:
        default:
            return "DISCONNECTED";
    }
}

// In v2 this compatibility field stores whether the inverter CAN keepalive is non-zero.
const char *inverter_status_to_text(bool inverter_online) {
    return inverter_online ? "ONLINE" : "OFFLINE";
}

// Translate simple 0/1 contactor state to text.
const char *binary_contactor_to_text(uint8_t engaged) {
    return engaged != 0U ? "CLOSED" : "OPEN";
}

// 0 -> normal, 1 -> paused, 2 -> fault.
lv_color_t screen_tint_color(int tint_state) {
    switch (tint_state) {
        case 2:
            return lv_color_hex(0xFF0000);
        case 1:
            return lv_color_hex(0xD6BB00);
        default:
            return lv_color_hex(0x000000);
    }
}

bool remote_state_is_fresh(const UiBatteryState &state) {
    return state.has_remote_state && ((millis() - state.last_remote_state_ms) <= REMOTE_STATE_TIMEOUT_MS);
}

// Same freshness helper for the smaller remote-state view.
bool remote_state_is_fresh(const UiRemoteStateView &state) {
    return state.has_remote_state && ((millis() - state.last_remote_state_ms) <= REMOTE_STATE_TIMEOUT_MS);
}

// Translate numeric fault level to text.
const char *fault_level_to_text(uint8_t level) {
    return (level < (sizeof(kEventLevels) / sizeof(kEventLevels[0]))) ? kEventLevels[level] : "UNKNOWN";
}

const char *fault_state_to_text(uint8_t state) {
    switch (state) {
        case 0: return "PENDING";
        case 1: return "INACTIVE";
        case 2: return "ACTIVE";
        case 3: return "LATCHED";
        default: return "UNKNOWN";
    }
}

// Translate numeric event id to text.
const char *fault_event_id_to_text(uint8_t event_id) {
    return (event_id < (sizeof(kEventNames) / sizeof(kEventNames[0]))) ? kEventNames[event_id] : "EVENT_UNKNOWN";
}

lv_obj_t *cell_min_label_for_slot(uint8_t slot);
lv_obj_t *cell_max_label_for_slot(uint8_t slot);
lv_obj_t *deviation_label_for_slot(uint8_t slot);
void clear_chart(uint8_t slot);
void set_label_text(lv_obj_t *obj, const char *text);

bool is_screen2_active() {
    return (ui_Bat1CellsScreen != nullptr) && (lv_scr_act() == ui_Bat1CellsScreen);
}

bool is_screen3_active() {
    return (ui_FaultsScreen != nullptr) && (lv_scr_act() == ui_FaultsScreen);
}

bool is_screen5_active() {
    return (ui_Bat2CellsScreen != nullptr) && (lv_scr_act() == ui_Bat2CellsScreen);
}

bool is_dual_main_active() {
    return (ui_DoubleBatScreen != nullptr) && (lv_scr_act() == ui_DoubleBatScreen);
}

bool is_single_main_active() {
    return (ui_SingleBatScreen != nullptr) && (lv_scr_act() == ui_SingleBatScreen);
}

bool is_dual_battery_mode() {
    // The v2 sender emits a frame for every configured slot, including undetected
    // batteries. Use the live CAN-alive indicator so the second pane appears only
    // when battery 2 is actually online.
    SharedBatteryState snapshot{};
    portENTER_CRITICAL(&g_state_mux);
    snapshot = g_shared_states[1];
    portEXIT_CRITICAL(&g_state_mux);
    return snapshot.has_status && snapshot.status.CAN_battery_still_alive != 0U &&
           ((millis() - snapshot.last_status_ms) <= UI_STALE_HOLD_MS);
}

// Battery Emulator v2 reports balancing independently from the BMS status.
// Keep the existing Bat*Status labels reserved for the BMS state and render
// this value in the dedicated SquareLine labels instead.
const char *balancing_status_to_text(int status) {
    switch (status) {
        case 3:
            return "Balancing ACTIVE";
        case 4:
            return "Balancing PENDING";
        case 2:
            return "Balancing OFF";
        case 1:
            return "Balancing ERROR";
        case 0:
        default:
            return "";  // Unknown or not reported: hide the label.
    }
}

void set_main_balancing_label(uint8_t slot, int balancing_status, bool dual_mode) {
    const char *text = balancing_status_to_text(balancing_status);
    if (!dual_mode) {
        if (slot == 0U && ui_BalancingSingle != nullptr) {
            lv_obj_set_style_text_font(ui_BalancingSingle, &lv_font_montserrat_8, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_align(ui_BalancingSingle, LV_ALIGN_BOTTOM_MID, 0, -27);
            set_label_text(ui_BalancingSingle, text);
        }
        return;
    }
    lv_obj_t *label = slot == 0U ? ui_Balancing1 : (slot == 1U ? ui_Balancing2 : nullptr);
    if (label != nullptr) {
        lv_obj_set_style_text_font(label, &lv_font_montserrat_8, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(label, LV_ALIGN_BOTTOM_MID, slot == 0U ? -75 : 75, -27);
        set_label_text(label, text);
    }
}

// Small wrapper around SquareLine's runtime screen change helper.
void load_screen(lv_obj_t **screen, void (*screen_init)(void)) {
    if (screen == nullptr) {
        return;
    }
    _ui_screen_change(screen, LV_SCR_LOAD_ANIM_NONE, 0, 0, screen_init);
}

// Choose the correct main screen based on current single/double battery mode.
void load_runtime_main_screen() {
    if (is_dual_battery_mode()) {
        load_screen(&ui_DoubleBatScreen, &ui_DoubleBatScreen_screen_init);
    } else {
        load_screen(&ui_SingleBatScreen, &ui_SingleBatScreen_screen_init);
    }
}

// Main-screen next button always goes to battery 1 cell view.
void screen_nav_main_next_event(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    load_screen(&ui_Bat1CellsScreen, &ui_Bat1CellsScreen_screen_init);
}

// Battery 1 back returns to whichever main screen is currently valid.
void screen_nav_bat1_back_event(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    load_runtime_main_screen();
}

// Battery 1 next goes either to battery 2 cells or directly to faults screen.
void screen_nav_bat1_next_event(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (is_dual_battery_mode()) {
        load_screen(&ui_Bat2CellsScreen, &ui_Bat2CellsScreen_screen_init);
    } else {
        load_screen(&ui_FaultsScreen, &ui_FaultsScreen_screen_init);
    }
}

// Battery 2 back always returns to battery 1 cells.
void screen_nav_bat2_back_event(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    load_screen(&ui_Bat1CellsScreen, &ui_Bat1CellsScreen_screen_init);
}

// Battery 2 next always goes to the shared faults screen.
void screen_nav_bat2_next_event(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    load_screen(&ui_FaultsScreen, &ui_FaultsScreen_screen_init);
}

// Faults back depends on whether battery 2 screen is relevant right now.
void screen_nav_faults_back_event(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (is_dual_battery_mode()) {
        load_screen(&ui_Bat2CellsScreen, &ui_Bat2CellsScreen_screen_init);
    } else {
        load_screen(&ui_Bat1CellsScreen, &ui_Bat1CellsScreen_screen_init);
    }
}

// Keep the CYD's reachable address visible without requiring the old Secret page.
void update_main_ip_label() {
    lv_obj_t *root = nullptr;
    if (is_single_main_active()) root = ui_SingleBatScreen;
    else if (is_dual_main_active()) root = ui_DoubleBatScreen;
    if (root == nullptr) return;
    if (g_main_ip_parent != root || g_main_ip_label == nullptr) {
        g_main_ip_label = lv_label_create(root);
        g_main_ip_parent = root;
        lv_obj_set_width(g_main_ip_label, LV_SIZE_CONTENT);
        lv_obj_set_height(g_main_ip_label, LV_SIZE_CONTENT);
        lv_obj_align(g_main_ip_label, LV_ALIGN_BOTTOM_LEFT, 5, -35);
        lv_obj_set_style_text_font(g_main_ip_label, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(g_main_ip_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    char text[24];
    const char *ip = screen_network_is_sta_connected() ? screen_network_sta_ip() : WiFi.softAPIP().toString().c_str();
    snprintf(text, sizeof(text), "IP %s", (ip != nullptr && ip[0] != '\0') ? ip : "--");
    set_label_text(g_main_ip_label, text);
}

// Show the CYD's own SSID and a slowly refreshed, filtered RSSI value.
void update_main_network_labels() {
    constexpr uint32_t RSSI_UPDATE_INTERVAL_MS = 3000U;
    static uint32_t last_rssi_update_ms = 0U;
    static int filtered_rssi = 0;
    static bool has_filtered_rssi = false;
    static bool previous_sta = false;
    static char rssi_text[16] = "dBm AP";

    const bool sta = screen_network_is_sta_connected();
    const char *ssid = sta ? screen_network_sta_ssid() : screen_network_ap_ssid();
    char text[40];
    snprintf(text, sizeof(text), "SSID %s", (ssid != nullptr && ssid[0] != '\0') ? ssid : "...");
    set_label_text(ui_SSID1, text);
    set_label_text(ui_SSID2, text);

    const uint32_t now = millis();
    const bool connection_changed = sta != previous_sta;
    if (sta) {
        if (!has_filtered_rssi || connection_changed || (now - last_rssi_update_ms) >= RSSI_UPDATE_INTERVAL_MS) {
            const int raw_rssi = screen_network_sta_rssi();
            filtered_rssi = has_filtered_rssi && !connection_changed
                                ? ((filtered_rssi * 3) + raw_rssi) / 4
                                : raw_rssi;
            has_filtered_rssi = true;
            last_rssi_update_ms = now;
            snprintf(rssi_text, sizeof(rssi_text), "%d dBm", filtered_rssi);
        }
    } else {
        has_filtered_rssi = false;
        snprintf(rssi_text, sizeof(rssi_text), "dBm AP");
    }
    previous_sta = sta;
    set_label_text(ui_DBM1, rssi_text);
    set_label_text(ui_DBM2, rssi_text);
}

// Remove generated SquareLine callbacks and replace them with runtime-aware ones.
// This keeps navigation correct even when single/double battery mode changes.
void install_navigation_overrides() {
    static lv_obj_t *last_double_root = nullptr;
    static lv_obj_t *last_single_root = nullptr;
    static lv_obj_t *last_bat1_root = nullptr;
    static lv_obj_t *last_bat2_root = nullptr;
    static lv_obj_t *last_faults_root = nullptr;

    if (ui_DoubleBatScreen != nullptr && last_double_root != ui_DoubleBatScreen && ui_NextButS1 != nullptr) {
        while (lv_obj_remove_event_cb(ui_NextButS1, nullptr)) {}
        lv_obj_add_event_cb(ui_NextButS1, screen_nav_main_next_event, LV_EVENT_CLICKED, nullptr);
        last_double_root = ui_DoubleBatScreen;
    }

    if (ui_SingleBatScreen != nullptr && last_single_root != ui_SingleBatScreen && ui_NextButS6 != nullptr) {
        while (lv_obj_remove_event_cb(ui_NextButS6, nullptr)) {}
        lv_obj_add_event_cb(ui_NextButS6, screen_nav_main_next_event, LV_EVENT_CLICKED, nullptr);
        last_single_root = ui_SingleBatScreen;
    }

    if (ui_Bat1CellsScreen != nullptr && last_bat1_root != ui_Bat1CellsScreen) {
        if (ui_BackButS2 != nullptr) {
            while (lv_obj_remove_event_cb(ui_BackButS2, nullptr)) {}
            lv_obj_add_event_cb(ui_BackButS2, screen_nav_bat1_back_event, LV_EVENT_CLICKED, nullptr);
        }
        if (ui_NextButS2 != nullptr) {
            while (lv_obj_remove_event_cb(ui_NextButS2, nullptr)) {}
            lv_obj_add_event_cb(ui_NextButS2, screen_nav_bat1_next_event, LV_EVENT_CLICKED, nullptr);
        }
        set_label_text(cell_min_label_for_slot(0U), "---mV");
        set_label_text(cell_max_label_for_slot(0U), "---mV");
        set_label_text(deviation_label_for_slot(0U), "---mV");
        clear_chart(0U);
        last_bat1_root = ui_Bat1CellsScreen;
    }

    if (ui_Bat2CellsScreen != nullptr && last_bat2_root != ui_Bat2CellsScreen) {
        if (ui_BackButS1 != nullptr) {
            while (lv_obj_remove_event_cb(ui_BackButS1, nullptr)) {}
            lv_obj_add_event_cb(ui_BackButS1, screen_nav_bat2_back_event, LV_EVENT_CLICKED, nullptr);
        }
        if (ui_NextButS4 != nullptr) {
            while (lv_obj_remove_event_cb(ui_NextButS4, nullptr)) {}
            lv_obj_add_event_cb(ui_NextButS4, screen_nav_bat2_next_event, LV_EVENT_CLICKED, nullptr);
        }
        set_label_text(cell_min_label_for_slot(1U), "---mV");
        set_label_text(cell_max_label_for_slot(1U), "---mV");
        set_label_text(deviation_label_for_slot(1U), "---mV");
        clear_chart(1U);
        last_bat2_root = ui_Bat2CellsScreen;
    }

    if (ui_FaultsScreen != nullptr && last_faults_root != ui_FaultsScreen) {
        if (ui_BackButS3 != nullptr) {
            while (lv_obj_remove_event_cb(ui_BackButS3, nullptr)) {}
            lv_obj_add_event_cb(ui_BackButS3, screen_nav_faults_back_event, LV_EVENT_CLICKED, nullptr);
        }
        last_faults_root = ui_FaultsScreen;
    }

}

// Fix up the active screen if the battery mode changed while the UI is already running.
void enforce_runtime_screen_flow() {
    if (is_dual_main_active() && !is_dual_battery_mode()) {
        load_screen(&ui_SingleBatScreen, &ui_SingleBatScreen_screen_init);
        return;
    }

    if (is_single_main_active() && is_dual_battery_mode()) {
        load_screen(&ui_DoubleBatScreen, &ui_DoubleBatScreen_screen_init);
        return;
    }

    if (is_screen5_active() && !is_dual_battery_mode()) {
        load_screen(&ui_Bat1CellsScreen, &ui_Bat1CellsScreen_screen_init);
    }
}

// Apply button colors only when something actually changed.
void set_button_colors(lv_obj_t *button, lv_color_t bg_color, lv_color_t text_color) {
    if (button == nullptr) {
        return;
    }

    const lv_color_t current_bg = lv_obj_get_style_bg_color(button, LV_PART_MAIN | LV_STATE_DEFAULT);
    const lv_color_t current_text = lv_obj_get_style_text_color(button, LV_PART_MAIN | LV_STATE_DEFAULT);
    const lv_coord_t current_border_width = lv_obj_get_style_border_width(button, LV_PART_MAIN | LV_STATE_DEFAULT);
    const lv_color_t current_border_color = lv_obj_get_style_border_color(button, LV_PART_MAIN | LV_STATE_DEFAULT);

    if (current_bg.full == bg_color.full &&
        current_text.full == text_color.full &&
        current_border_width == 2 &&
        current_border_color.full == lv_color_black().full) {
        return;
    }

    lv_obj_set_style_bg_color(button, bg_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(button, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(button, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(button, text_color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

// Drive the RGB status LED on the CYD board.
void set_status_led(uint8_t red, uint8_t green, uint8_t blue) {
    analogWrite(RED_LED_PIN, 255 - red);
    analogWrite(GREEN_LED_PIN, 255 - green);
    analogWrite(BLUE_LED_PIN, 255 - blue);
}

// Update LED color for current battery state.
void update_led_for_status(int status) {
    if (status == g_last_led_status) {
        return;
    }

    g_last_led_status = status;

    switch (status) {
        case BMS_STANDBY:
            set_status_led(255, 180, 0);
            break;
        case BMS_ACTIVE:
            set_status_led(0, 255, 0);
            break;
        case BMS_FAULT:
            set_status_led(255, 0, 0);
            break;
        case BMS_DISCONNECTED:
            set_status_led(128, 0, 255);
            break;
        default:
            set_status_led(0, 0, 255);
            break;
    }
}

// Screen-level warning/fault states must have priority over normal battery colors.
// This keeps the RGB LED aligned with the large background tint seen on the UI.
void update_led_for_state(const UiRemoteStateView &remote_state_view, int battery_status) {
    if (remote_state_is_fresh(remote_state_view)) {
        if (remote_state_view.remote_state.system_status == SYS_FAULT) {
            update_led_for_status(BMS_FAULT);
            return;
        }

        if (remote_state_view.remote_state.pause_request_on != 0U ||
            remote_state_view.remote_state.pause_status != 0U) {
            update_led_for_status(BMS_STANDBY);
            return;
        }
    }

    update_led_for_status(battery_status);
}

// Helper overload when the caller already has the full battery snapshot.
void update_led_for_state(const UiBatteryState &state) {
    UiRemoteStateView remote_view{};
    remote_view.has_remote_state = state.has_remote_state;
    remote_view.last_remote_state_ms = state.last_remote_state_ms;
    remote_view.remote_state = state.remote_state;
    remote_view.pending_command_id = state.pending_command_id;
    remote_view.pending_command_sent_ms = state.pending_command_sent_ms;
    remote_view.command_pending = state.command_pending;
    update_led_for_state(remote_view, state.status.real_bms_status);
}

// Apply whole-screen background tint to one LVGL screen object.
void apply_screen_tint_to_obj(lv_obj_t *screen, lv_color_t color) {
    if (screen == nullptr) {
        return;
    }

    const lv_color_t current_bg = lv_obj_get_style_bg_color(screen, LV_PART_MAIN | LV_STATE_DEFAULT);
    const lv_opa_t current_opa = lv_obj_get_style_bg_opa(screen, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (current_bg.full == color.full && current_opa == LV_OPA_COVER) {
        return;
    }

    lv_obj_set_style_bg_color(screen, color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
}

// Decide whether screens should be normal, warning yellow, or fault red.
void update_screen_tint(const UiRemoteStateView &state) {
    int tint_state = 0;
    if (remote_state_is_fresh(state)) {
        if (state.remote_state.system_status == SYS_FAULT) {
            tint_state = 2;
        } else if (state.remote_state.pause_request_on != 0U || state.remote_state.pause_status != 0U) {
            tint_state = 1;
        }
    }

    if (tint_state == g_last_screen_tint) {
        return;
    }
    g_last_screen_tint = tint_state;

    const lv_color_t tint = screen_tint_color(tint_state);
    apply_screen_tint_to_obj(ui_DoubleBatScreen, tint);
    apply_screen_tint_to_obj(ui_SingleBatScreen, tint);
    apply_screen_tint_to_obj(ui_Bat1CellsScreen, tint);
    apply_screen_tint_to_obj(ui_Bat2CellsScreen, tint);
    apply_screen_tint_to_obj(ui_FaultsScreen, tint);
}

// Overload that extracts just the remote-state subset from a full battery snapshot.
void update_screen_tint(const UiBatteryState &state) {
    UiRemoteStateView remote_view{};
    remote_view.has_remote_state = state.has_remote_state;
    remote_view.last_remote_state_ms = state.last_remote_state_ms;
    remote_view.remote_state = state.remote_state;
    remote_view.pending_command_id = state.pending_command_id;
    remote_view.pending_command_sent_ms = state.pending_command_sent_ms;
    remote_view.command_pending = state.command_pending;
    update_screen_tint(remote_view);
}

// Safe label setter that avoids redraws when text is unchanged.
void set_label_text(lv_obj_t *obj, const char *text) {
    if (obj != nullptr) {
        const char *safe_text = (text != nullptr) ? text : "";
        const char *current_text = lv_label_get_text(obj);
        if (current_text != nullptr && strcmp(current_text, safe_text) == 0) {
            return;
        }
        lv_label_set_text(obj, safe_text);
    }
}

// Safe textarea setter that avoids redraws when text is unchanged.
void set_textarea_text(lv_obj_t *obj, const char *text) {
    if (obj == nullptr) {
        return;
    }

    const char *safe_text = (text != nullptr) ? text : "";
    const char *current_text = lv_textarea_get_text(obj);
    if (current_text != nullptr && strcmp(current_text, safe_text) == 0) {
        return;
    }

    const lv_coord_t scroll_y = lv_obj_get_scroll_y(obj);
    lv_textarea_set_text(obj, safe_text);
    lv_obj_update_layout(obj);
    lv_obj_scroll_to_y(obj, scroll_y, LV_ANIM_OFF);
}

// Small printf-style helper for labels.
void set_label_fmt(lv_obj_t *obj, const char *fmt, ...) {
    if (obj == nullptr) {
        return;
    }

    char buffer[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    set_label_text(obj, buffer);
}

// Render signed tenths like 12.3 A or -4.5 A.
void set_label_signed_tenths(lv_obj_t *obj, int32_t value, const char *unit) {
    if (obj == nullptr) {
        return;
    }

    const char *sign = value < 0 ? "-" : "";
    const int32_t absolute_value = std::abs(value);
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%s%ld.%ld %s",
             sign,
             static_cast<long>(absolute_value / 10),
             static_cast<long>(absolute_value % 10),
             unit);
    set_label_text(obj, buffer);
}

// Render watts as kW with two decimal places.
void set_label_signed_hundredths_from_watts(lv_obj_t *obj, int32_t value_watts) {
    if (obj == nullptr) {
        return;
    }

    const char *sign = value_watts < 0 ? "-" : "";
    const int32_t absolute_value = std::abs(value_watts);
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%s%ld.%02ld kW",
             sign,
             static_cast<long>(absolute_value / 1000),
             static_cast<long>((absolute_value % 1000) / 10));
    set_label_text(obj, buffer);
}

// Show isolation resistance as kOhm or MOhm depending on size.
void set_label_isolation_resistance(lv_obj_t *obj, uint32_t isolation_kohm) {
    if (obj == nullptr) {
        return;
    }

    if (isolation_kohm == 0U) {
        set_label_text(obj, "---");
        return;
    }

    char buffer[32];
    if (isolation_kohm >= 1000U) {
        snprintf(buffer,
                 sizeof(buffer),
                 "%lu.%02lu MOhm",
                 static_cast<unsigned long>(isolation_kohm / 1000U),
                 static_cast<unsigned long>((isolation_kohm % 1000U) / 10U));
    } else {
        snprintf(buffer, sizeof(buffer), "%lu kOhm", static_cast<unsigned long>(isolation_kohm));
    }
    set_label_text(obj, buffer);
}

// Render battery min/max temperature in "min/max°C" format.
void set_label_temp_min_max(lv_obj_t *obj, int16_t min_temp_dC, int16_t max_temp_dC) {
    if (obj == nullptr) {
        return;
    }

    char buffer[32];
    snprintf(buffer,
             sizeof(buffer),
             "%d/%d\xC2\xB0""C",
             static_cast<int>(min_temp_dC / 10),
             static_cast<int>(max_temp_dC / 10));
    set_label_text(obj, buffer);
}

// Clear local pending-command flag once the matching ACK comes back from the emulator.
void clear_pending_command_if_ack_matches(SharedBatteryState &shared_state,
                                          const BATTERY_REMOTE_STATE_TYPE &remote_state) {
    if (!shared_state.command_pending) {
        return;
    }

    if (remote_state.ack_sequence != shared_state.pending_command_sequence) {
        return;
    }

    if (remote_state.ack_command_id != shared_state.pending_command_id) {
        return;
    }

    shared_state.command_pending = false;
}

// Select the right chart object for a battery slot.
lv_obj_t *chart_for_slot(uint8_t slot) {
    return (slot == 0U) ? ui_Chart2 : ui_Chart1;
}

// Select the right min-cell label for a battery slot.
lv_obj_t *cell_min_label_for_slot(uint8_t slot) {
    return (slot == 0U) ? ui_CellMinValue : ui_CellMinValue1;
}

// Select the right max-cell label for a battery slot.
lv_obj_t *cell_max_label_for_slot(uint8_t slot) {
    return (slot == 0U) ? ui_CellMaxValue : ui_CellMaxValue1;
}

// Select the right deviation label for a battery slot.
lv_obj_t *deviation_label_for_slot(uint8_t slot) {
    return (slot == 0U) ? ui_DevValue : ui_DevValue1;
}

// Bind chart data arrays the first time a chart is used.
void ensure_chart_ready(uint8_t slot) {
    lv_obj_t *chart = chart_for_slot(slot);
    if (chart == nullptr || g_chart_series[slot] != nullptr) {
        return;
    }

    g_chart_series[slot] = lv_chart_get_series_next(chart, nullptr);
    if (g_chart_series[slot] == nullptr) {
        return;
    }

    lv_chart_set_point_count(chart, MAX_UI_CHART_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 2500, 4500);
    lv_chart_set_ext_y_array(chart, g_chart_series[slot], g_cell_chart_values[slot]);
    lv_chart_refresh(chart);
}

// Clear a cells chart when we no longer have fresh data for that battery.
void clear_chart(uint8_t slot) {
    lv_obj_t *chart = chart_for_slot(slot);
    ensure_chart_ready(slot);
    if (chart == nullptr || g_chart_series[slot] == nullptr) {
        return;
    }

    for (uint16_t i = 0; i < MAX_UI_CHART_POINTS; ++i) {
        g_cell_chart_values[slot][i] = 0;
    }
    lv_chart_set_point_count(chart, MAX_UI_CHART_POINTS);
    lv_chart_refresh(chart);
}

// Push current cell voltages into the visible battery chart.
// 16-bit data is preferred when it is fresh; otherwise 8-bit cells are used.
void update_chart_from_cells(uint8_t slot, const UiBatteryState &state) {
    if ((slot == 0U && !is_screen2_active()) || (slot == 1U && !is_screen5_active())) {
        return;
    }

    lv_obj_t *chart = chart_for_slot(slot);
    ensure_chart_ready(slot);
    if (chart == nullptr || g_chart_series[slot] == nullptr) {
        return;
    }

    const bool use_16bit_cells =
        state.has_cells_16bit && ((millis() - state.last_cells_16bit_ms) <= HIGH_RES_CELL_TIMEOUT_MS);
    const bool use_8bit_cells =
        state.has_cells && ((millis() - state.last_cells_ms) <= HIGH_RES_CELL_TIMEOUT_MS);

    if (!use_16bit_cells && !use_8bit_cells) {
        clear_chart(slot);
        return;
    }

    uint16_t point_count = use_16bit_cells ? state.cell16_total_cells : state.cells.number_of_cells;
    if (point_count == 0) {
        return;
    }
    if (point_count > MAX_UI_CHART_POINTS) {
        point_count = MAX_UI_CHART_POINTS;
    }

    uint16_t min_mV = 5000;
    uint16_t max_mV = 0;

    for (uint16_t i = 0; i < point_count; ++i) {
        const uint16_t cell_mV = use_16bit_cells ? state.cells_16bit_mV[i]
                                                 : static_cast<uint16_t>(state.cells.cell_voltages_mV[i]) * 20U;
        g_cell_chart_values[slot][i] = static_cast<lv_coord_t>(cell_mV);
        if (cell_mV < min_mV) {
            min_mV = cell_mV;
        }
        if (cell_mV > max_mV) {
            max_mV = cell_mV;
        }
    }

    for (uint16_t i = point_count; i < MAX_UI_CHART_POINTS; ++i) {
        g_cell_chart_values[slot][i] = 0;
    }

    uint16_t range_min = (min_mV > 100) ? static_cast<uint16_t>(min_mV - 100U) : min_mV;
    uint16_t range_max = static_cast<uint16_t>(max_mV + 100U);
    if (range_max <= range_min) {
        range_min = 2500;
        range_max = 4500;
    }

    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, range_min, range_max);
    lv_chart_set_point_count(chart, point_count);
    lv_chart_refresh(chart);
}

// Update the shared faults screen from either structured events or fallback text.
void apply_fault_text_ui(const UiBatteryState &state) {
    if (ui_TextArea1 == nullptr || !is_screen3_active()) {
        return;
    }

    if (state.has_fault_events && (millis() - state.last_fault_events_ms) <= FAULT_EVENTS_TIMEOUT_MS) {
        if (state.fault_event_count == 0) {
            set_textarea_text(ui_TextArea1, "No faults");
            return;
        }

        char buffer[2048];
        size_t offset = 0;
        buffer[0] = '\0';

        for (uint8_t i = 0; i < state.fault_event_count; ++i) {
            const auto &event = state.fault_events[i];
            const int written = snprintf(buffer + offset,
                                         sizeof(buffer) - offset,
                                         "#%u  [%s]\n%s\nData: %u   Count: %u\n%s",
                                         static_cast<unsigned>(i + 1U),
                                         fault_level_to_text(event.level),
                                         fault_event_id_to_text(event.event_id),
                                         static_cast<unsigned>(event.data),
                                         static_cast<unsigned>(event.count),
                                         (i + 1U < state.fault_event_count) ? "------------------------\n" : "");
            if (written <= 0 || static_cast<size_t>(written) >= (sizeof(buffer) - offset)) {
                break;
            }
            offset += static_cast<size_t>(written);
        }

        set_textarea_text(ui_TextArea1, buffer);
        return;
    }

    if (!state.has_fault_text || (millis() - state.last_fault_text_ms) > FAULT_TEXT_TIMEOUT_MS) {
        set_textarea_text(ui_TextArea1, "No faults sent");
        return;
    }

    if (state.fault_text[0] == '\0') {
        set_textarea_text(ui_TextArea1, "No faults");
        return;
    }

    set_textarea_text(ui_TextArea1, state.fault_text);
}

// Show placeholder values while battery packets are missing or stale.
void apply_waiting_ui(uint8_t slot) {
    UiRemoteStateView remote_snapshot{};
    SharedBatteryState &shared_state = primary_shared_state();
    portENTER_CRITICAL(&g_state_mux);
    remote_snapshot.has_remote_state = shared_state.has_remote_state;
    remote_snapshot.remote_state = shared_state.remote_state;
    remote_snapshot.last_remote_state_ms = shared_state.last_remote_state_ms;
    remote_snapshot.command_pending = shared_state.command_pending;
    remote_snapshot.pending_command_id = shared_state.pending_command_id;
    remote_snapshot.pending_command_sent_ms = shared_state.pending_command_sent_ms;
    portEXIT_CRITICAL(&g_state_mux);

    if (g_ui_showing_waiting[slot]) {
        if (slot == 0U) {
            update_screen_tint(remote_snapshot);
        }
        set_main_balancing_label(slot, 0, is_dual_battery_mode());
        return;
    }

    g_ui_showing_waiting[slot] = true;

    const bool dual_mode = is_dual_battery_mode();

    if (slot == 0U) {
        if (dual_mode) {
            set_label_text(ui_SOCValue, "---%");
            set_label_text(ui_SOHValue, "---%");
            set_label_text(ui_VoltageValue, "--- V");
            set_label_text(ui_CurrentValue, "--- A");
            set_label_text(ui_PowerValue, "--- kW");
            set_label_text(ui_RezistanceValue, "---");
            set_label_text(ui_TempMinMaxValueBat1, "--/--°C");
            set_label_text(ui_StatusValue, "Waiting");
            set_label_text(ui_Bat1Status, "Battery: Waiting");
            if (ui_SOCArc != nullptr && lv_arc_get_value(ui_SOCArc) != 0) {
                lv_arc_set_value(ui_SOCArc, 0);
            }
        } else {
            set_label_text(ui_SOCValue3, "---%");
            set_label_text(ui_SOHValue3, "---%");
            set_label_text(ui_VoltageValue3, "--- V");
            set_label_text(ui_CurrentValue3, "--- A");
            set_label_text(ui_PowerValue3, "--- kW");
            set_label_text(ui_RezistanceValue3, "---");
            set_label_text(ui_TempMinMaxValueSingleBat, "--/--°C");
            set_label_text(ui_StatusValue2, "Waiting");
            set_label_text(ui_SingleBatStatus, "Battery: Waiting");
            if (ui_SOCArc3 != nullptr && lv_arc_get_value(ui_SOCArc3) != 0) {
                lv_arc_set_value(ui_SOCArc3, 0);
            }
        }
        if (is_screen2_active()) {
            clear_chart(slot);
        }
        update_led_for_state(remote_snapshot, -1);
        if (ui_TextArea1 != nullptr && is_screen3_active()) {
            set_textarea_text(ui_TextArea1, "No faults sent");
        }
        update_screen_tint(remote_snapshot);
    } else {
        set_label_text(ui_SOCValue2, "---%");
        set_label_text(ui_SOHValue2, "---%");
        set_label_text(ui_VoltageValue2, "--- V");
        set_label_text(ui_CurrentValue2, "--- A");
        set_label_text(ui_PowerValue2, "--- kW");
        set_label_isolation_resistance(ui_RezistanceValue2, 0);
        set_label_text(ui_TempMinMaxValueBat2, "--/--°C");
        set_label_text(ui_Bat2Status, "Battery: Waiting");
        if (ui_SOCArc2 != nullptr && lv_arc_get_value(ui_SOCArc2) != 0) {
            lv_arc_set_value(ui_SOCArc2, 0);
        }
    }

    set_main_balancing_label(slot, 0, dual_mode);

    set_label_text(cell_min_label_for_slot(slot), "---mV");
    set_label_text(cell_max_label_for_slot(slot), "---mV");
    set_label_text(deviation_label_for_slot(slot), "---mV");
    clear_chart(slot);

    if (slot == 0U && !g_waiting_log_printed) {
        Serial.println("ESP-NOW: waiting for BAT_STATUS packets...");
        g_waiting_log_printed = true;
    }
}

// Update the battery status text shown on the correct main screen.
void set_main_battery_status_labels(uint8_t slot, int status, uint8_t can_alive, bool dual_mode) {
    bool has_contactors = false;
    uint8_t contactors = 0U;
    uint32_t contactors_update_ms = 0U;
    portENTER_CRITICAL(&g_state_mux);
    has_contactors = g_shared_states[0].has_remote_state;
    contactors = g_shared_states[0].remote_state.contactors_engaged;
    contactors_update_ms = g_shared_states[0].last_remote_state_ms;
    portEXIT_CRITICAL(&g_state_mux);
    has_contactors = has_contactors && ((millis() - contactors_update_ms) <= REMOTE_STATE_TIMEOUT_MS);

    char text[40];
    if (has_contactors) {
        switch (contactors) {
            case 1U:
                snprintf(text, sizeof(text), "Battery: CONTACTORS CLOSED");
                break;
            case 2U:
                snprintf(text, sizeof(text), "Battery: OPEN (FAULT)");
                break;
            case 3U:
                snprintf(text, sizeof(text), "Battery: PRECHARGE");
                break;
            case 0U:
            default:
                snprintf(text, sizeof(text), "Battery: CONTACTORS OPEN");
                break;
        }
    } else if (status == BMS_DISCONNECTED && can_alive == 0U) {
        snprintf(text, sizeof(text), "Battery: DISCONNECTED");
    } else {
        snprintf(text, sizeof(text), "Battery: %s", status_to_text(status));
    }

    if (slot == 0U) {
        if (dual_mode) {
            if (ui_Bat1Status != nullptr) {
                lv_obj_set_style_text_font(ui_Bat1Status, &lv_font_montserrat_8, LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            set_label_text(ui_Bat1Status, text);
        } else {
            set_label_text(ui_SingleBatStatus, text);
        }
        return;
    }

    if (dual_mode) {
        if (ui_Bat2Status != nullptr) {
            lv_obj_set_style_text_font(ui_Bat2Status, &lv_font_montserrat_8, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        set_label_text(ui_Bat2Status, text);
    }
}

// Update inverter status text on the active main-screen layout.
void set_main_inverter_status(const UiBatteryState &state, bool dual_mode) {
    if (state.has_remote_state && ((millis() - state.last_remote_state_ms) <= REMOTE_STATE_TIMEOUT_MS)) {
        const char *status_text =
            inverter_status_to_text(state.remote_state.inverter_allows_contactor_closing != 0U);
        if (dual_mode) {
            set_label_text(ui_StatusValue, status_text);
        } else {
            set_label_text(ui_StatusValue2, status_text);
        }
    } else {
        if (dual_mode) {
            set_label_text(ui_StatusValue, "WAITING");
        } else {
            set_label_text(ui_StatusValue2, "WAITING");
        }
    }
}

// Render one battery slot into the UI using the latest fresh packet snapshot.
void apply_ui_state(uint8_t slot, const UiBatteryState &state) {
    const uint32_t age = millis() - state.last_status_ms;
    if (!state.has_status || age > UI_STALE_HOLD_MS) {
        apply_waiting_ui(slot);
        return;
    }

    g_ui_showing_waiting[slot] = false;
    const uint16_t deviation_mV = state.status.cell_max_voltage_mV - state.status.cell_min_voltage_mV;

    const bool dual_mode = is_dual_battery_mode();

    if (slot == 0U) {
        if (dual_mode) {
            set_label_fmt(ui_SOCValue, "%u.%02u%%", state.status.reported_soc / 100U, state.status.reported_soc % 100U);
            set_label_fmt(ui_SOHValue, "%u.%02u%%", state.status.soh_pptt / 100U, state.status.soh_pptt % 100U);
            set_label_fmt(ui_VoltageValue, "%u.%u V", state.status.voltage_dV / 10U, state.status.voltage_dV % 10U);
            set_label_signed_tenths(ui_CurrentValue, state.status.reported_current_dA, "A");
            set_label_signed_hundredths_from_watts(ui_PowerValue, state.status.active_power_W);
            set_label_fmt(ui_RezistanceValue, "%u mV", deviation_mV);
            set_label_temp_min_max(ui_TempMinMaxValueBat1,
                                   state.status.temperature_min_dC,
                                   state.status.temperature_max_dC);
            if (ui_SOCArc != nullptr) {
                const int arc_value = state.status.reported_soc / 100U;
                if (lv_arc_get_value(ui_SOCArc) != arc_value) {
                    lv_arc_set_value(ui_SOCArc, arc_value);
                }
            }
        } else {
            set_label_fmt(ui_SOCValue3, "%u.%02u%%", state.status.reported_soc / 100U, state.status.reported_soc % 100U);
            set_label_fmt(ui_SOHValue3, "%u.%02u%%", state.status.soh_pptt / 100U, state.status.soh_pptt % 100U);
            set_label_fmt(ui_VoltageValue3, "%u.%u V", state.status.voltage_dV / 10U, state.status.voltage_dV % 10U);
            set_label_signed_tenths(ui_CurrentValue3, state.status.reported_current_dA, "A");
            set_label_signed_hundredths_from_watts(ui_PowerValue3, state.status.active_power_W);
            set_label_fmt(ui_RezistanceValue3, "%u mV", deviation_mV);
            set_label_temp_min_max(ui_TempMinMaxValueSingleBat,
                                   state.status.temperature_min_dC,
                                   state.status.temperature_max_dC);
            if (ui_SOCArc3 != nullptr) {
                const int arc_value = state.status.reported_soc / 100U;
                if (lv_arc_get_value(ui_SOCArc3) != arc_value) {
                    lv_arc_set_value(ui_SOCArc3, arc_value);
                }
            }
        }
        set_main_inverter_status(state, dual_mode);
        set_main_battery_status_labels(slot,
                                       state.status.real_bms_status,
                                       state.status.CAN_battery_still_alive,
                                       dual_mode);
        set_main_balancing_label(slot, state.status.balancing_status, dual_mode);
        update_led_for_state(state);
        apply_fault_text_ui(state);
        update_screen_tint(state);
    } else {
        set_label_fmt(ui_SOCValue2, "%u.%02u%%", state.status.reported_soc / 100U, state.status.reported_soc % 100U);
        set_label_fmt(ui_SOHValue2, "%u.%02u%%", state.status.soh_pptt / 100U, state.status.soh_pptt % 100U);
        set_label_fmt(ui_VoltageValue2, "%u.%u V", state.status.voltage_dV / 10U, state.status.voltage_dV % 10U);
        set_label_signed_tenths(ui_CurrentValue2, state.status.reported_current_dA, "A");
        set_label_signed_hundredths_from_watts(ui_PowerValue2, state.status.active_power_W);
        set_label_fmt(ui_RezistanceValue2, "%u mV", deviation_mV);
        set_label_temp_min_max(ui_TempMinMaxValueBat2,
                               state.status.temperature_min_dC,
                               state.status.temperature_max_dC);
        set_main_battery_status_labels(slot,
                                       state.status.real_bms_status,
                                       state.status.CAN_battery_still_alive,
                                       dual_mode);
        set_main_balancing_label(slot, state.status.balancing_status, dual_mode);
        if (ui_SOCArc2 != nullptr) {
            const int arc_value = state.status.reported_soc / 100U;
            if (lv_arc_get_value(ui_SOCArc2) != arc_value) {
                lv_arc_set_value(ui_SOCArc2, arc_value);
            }
        }
    }

    if ((slot == 0U && is_screen2_active()) || (slot == 1U && is_screen5_active())) {
        set_label_fmt(cell_min_label_for_slot(slot), "%umV", state.status.cell_min_voltage_mV);
        set_label_fmt(cell_max_label_for_slot(slot), "%umV", state.status.cell_max_voltage_mV);
        set_label_fmt(deviation_label_for_slot(slot), "%umV", deviation_mV);
    }

    update_chart_from_cells(slot, state);

    const uint32_t now = millis();
    if (slot == 0U && now - g_last_ui_log_ms >= RX_LOG_INTERVAL_MS) {
        g_last_ui_log_ms = now;
        Serial.printf("UI updated: SOC=%u.%02u%% SOH=%u.%02u%% V=%u.%u I=%d.%d P=%ldW Dev=%umV Status=%s\n",
                      state.status.reported_soc / 100U,
                      state.status.reported_soc % 100U,
                      state.status.soh_pptt / 100U,
                      state.status.soh_pptt % 100U,
                      state.status.voltage_dV / 10U,
                      state.status.voltage_dV % 10U,
                      std::abs(state.status.reported_current_dA) / 10,
                      std::abs(state.status.reported_current_dA) % 10,
                      static_cast<long>(state.status.active_power_W),
                      deviation_mV,
                      status_to_text(state.status.real_bms_status));
    }

    if (slot == 0U) {
        g_waiting_log_printed = false;
    }
}

// Use normal UI when packets are fresh, otherwise fall back to waiting placeholders.
void apply_cached_or_waiting_ui(uint8_t slot, const UiBatteryState &state) {
    if (state.has_status && (millis() - state.last_status_ms) <= UI_STALE_HOLD_MS) {
        apply_ui_state(slot, state);
        return;
    }
    apply_waiting_ui(slot);
}

// ESP-NOW receive callback.
// This only parses packets into shared state; LVGL updates happen later in the main loop.
void on_data_recv(const uint8_t *mac, const uint8_t *incoming_data, int len) {
    (void)mac;

    if (incoming_data == nullptr || len < static_cast<int>(ESPNOW_HEADER_SIZE)) {
        return;
    }

    ESPNOW_HEADER header{};
    memcpy(&header, incoming_data, sizeof(header));

    const int slot = battery_slot_from_id(header.battery_id);
    if (slot < 0) {
        return;
    }
    SharedBatteryState &shared_state = g_shared_states[slot];

    // Copy just the packet payload into shared state under ISR-safe lock.
    portENTER_CRITICAL_ISR(&g_state_mux);
    shared_state.rx_packet_count++;
    shared_state.rx_event_pending = true;
    shared_state.last_message_type = header.esp_message_type;
    shared_state.last_message_len = len;
    shared_state.emulator_id = header.emulator_id;
    shared_state.battery_id = header.battery_id;
    shared_state.last_packet_ms = millis();

    // BAT_INFO updates battery design/static info.
    if (header.esp_message_type == BAT_INFO &&
               len > static_cast<int>(ESPNOW_HEADER_SIZE)) {
        const size_t payload_len = static_cast<size_t>(len) - ESPNOW_HEADER_SIZE;
        memset(&shared_state.info, 0, sizeof(shared_state.info));
        memcpy(&shared_state.info,
               incoming_data + ESPNOW_HEADER_SIZE,
               (sizeof(shared_state.info) < payload_len) ? sizeof(shared_state.info) : payload_len);
        shared_state.has_info = true;
        shared_state.last_info_ms = millis();
        shared_state.dirty = true;
    // BAT_STATUS updates live pack measurements.
    } else if (header.esp_message_type == BAT_STATUS &&
               len > static_cast<int>(ESPNOW_HEADER_SIZE)) {
        const size_t payload_len = static_cast<size_t>(len) - ESPNOW_HEADER_SIZE;
        memset(&shared_state.status, 0, sizeof(shared_state.status));
        memcpy(&shared_state.status,
               incoming_data + ESPNOW_HEADER_SIZE,
               (sizeof(shared_state.status) < payload_len) ? sizeof(shared_state.status) : payload_len);
        shared_state.has_status = true;
        shared_state.last_status_ms = millis();
        shared_state.dirty = true;
    // BAT_AUX_INFO carries isolation resistance and similar secondary values.
    } else if (header.esp_message_type == BAT_AUX_INFO &&
               len >= static_cast<int>(ESPNOW_HEADER_SIZE + sizeof(BATTERY_AUX_INFO_TYPE))) {
        memcpy(&shared_state.aux_info,
               incoming_data + ESPNOW_HEADER_SIZE,
               sizeof(BATTERY_AUX_INFO_TYPE));
        shared_state.has_aux_info = true;
        shared_state.last_aux_info_ms = millis();
        shared_state.dirty = true;
    // BAT_REMOTE_NET_INFO is parsed even though the current UI does not show it.
    } else if (header.esp_message_type == BAT_REMOTE_NET_INFO &&
               len >= static_cast<int>(ESPNOW_HEADER_SIZE + sizeof(BATTERY_REMOTE_NET_INFO_TYPE))) {
        memcpy(&shared_state.remote_net_info,
               incoming_data + ESPNOW_HEADER_SIZE,
               sizeof(BATTERY_REMOTE_NET_INFO_TYPE));
        shared_state.has_remote_net_info = true;
        shared_state.dirty = true;
    // BAT_CELL_STATUS holds compact 8-bit cell bars.
    } else if (header.esp_message_type == BAT_CELL_STATUS &&
               len > static_cast<int>(ESPNOW_HEADER_SIZE)) {
        const size_t payload_len = static_cast<size_t>(len) - ESPNOW_HEADER_SIZE;
        memset(&shared_state.cells, 0, sizeof(shared_state.cells));
        memcpy(&shared_state.cells,
               incoming_data + ESPNOW_HEADER_SIZE,
               (sizeof(shared_state.cells) < payload_len) ? sizeof(shared_state.cells) : payload_len);
        shared_state.has_cells = true;
        shared_state.last_cells_ms = millis();
        shared_state.dirty = true;
    // BAT_CELL_STATUS_16BIT carries chunked full-resolution cell data.
    } else if (header.esp_message_type == BAT_CELL_STATUS_16BIT &&
               len >= static_cast<int>(ESPNOW_HEADER_SIZE + 4U)) {
        const auto* chunk =
            reinterpret_cast<const BATTERY_CELL_STATUS_16BIT_CHUNK_TYPE*>(incoming_data + ESPNOW_HEADER_SIZE);
        const size_t payload_len = static_cast<size_t>(len) - ESPNOW_HEADER_SIZE;
        const size_t available_cell_bytes = payload_len - 4U;
        const uint8_t max_cells_in_message = static_cast<uint8_t>(available_cell_bytes / sizeof(uint16_t));
        const uint8_t cells_in_chunk = (chunk->cells_in_chunk < max_cells_in_message) ? chunk->cells_in_chunk
                                                                                       : max_cells_in_message;
        const uint8_t total_cells = (chunk->total_cells <= MAX_UI_CHART_POINTS) ? chunk->total_cells
                                                                                 : MAX_UI_CHART_POINTS;

        if (chunk->transfer_id != shared_state.cell16_transfer_id) {
            shared_state.cell16_transfer_id = chunk->transfer_id;
            shared_state.cell16_total_cells = total_cells;
            shared_state.cell16_expected_chunks = static_cast<uint8_t>((total_cells + cells_in_chunk - 1U) /
                                                                         (cells_in_chunk == 0 ? 1U : cells_in_chunk));
            shared_state.cell16_received_chunks_mask = 0;
            memset(shared_state.cells_16bit_mV, 0, sizeof(shared_state.cells_16bit_mV));
        }

        for (uint8_t i = 0; i < cells_in_chunk; ++i) {
            const uint16_t target_index = static_cast<uint16_t>(chunk->start_index) + i;
            if (target_index >= MAX_UI_CHART_POINTS) {
                break;
            }
            shared_state.cells_16bit_mV[target_index] = chunk->cell_voltages_mV[i];
        }

        if (cells_in_chunk > 0) {
            const uint8_t chunk_slot =
                (cells_in_chunk == 0) ? 0U : static_cast<uint8_t>(chunk->start_index / cells_in_chunk);
            if (chunk_slot < 8U) {
                shared_state.cell16_received_chunks_mask |= (1U << chunk_slot);
            }
        }

        shared_state.last_cells_16bit_ms = millis();
        shared_state.has_cells_16bit = true;
        shared_state.dirty = true;
    // BAT_FAULT_TEXT reassembles a larger plain-text faults message.
    } else if (header.esp_message_type == BAT_FAULT_TEXT &&
               len >= static_cast<int>(ESPNOW_HEADER_SIZE + 4U)) {
        const auto* chunk =
            reinterpret_cast<const BATTERY_FAULT_TEXT_CHUNK_TYPE*>(incoming_data + ESPNOW_HEADER_SIZE);
        const size_t payload_len = static_cast<size_t>(len) - ESPNOW_HEADER_SIZE;
        const size_t available_text_bytes = payload_len - 4U;
        const uint8_t text_length = (chunk->text_length < available_text_bytes) ? chunk->text_length
                                                                                 : static_cast<uint8_t>(available_text_bytes);

        if (chunk->transfer_id != shared_state.fault_transfer_id) {
            shared_state.fault_transfer_id = chunk->transfer_id;
            shared_state.fault_expected_chunks = chunk->total_chunks;
            shared_state.fault_received_chunks_mask = 0;
            memset(shared_state.fault_text, 0, sizeof(shared_state.fault_text));
        }

        const size_t text_offset = static_cast<size_t>(chunk->chunk_index) * 240U;
        if (text_offset < sizeof(shared_state.fault_text)) {
            const size_t bytes_to_copy =
                (text_offset + text_length <= sizeof(shared_state.fault_text) - 1U)
                    ? text_length
                    : (sizeof(shared_state.fault_text) - 1U - text_offset);
            memcpy(shared_state.fault_text + text_offset, chunk->text, bytes_to_copy);
            shared_state.fault_text[text_offset + bytes_to_copy] = '\0';
        }

        if (chunk->chunk_index < 32U) {
            shared_state.fault_received_chunks_mask |= (1UL << chunk->chunk_index);
        }
        shared_state.last_fault_text_ms = millis();
        shared_state.has_fault_text = true;
        shared_state.dirty = true;
    // BAT_FAULT_EVENTS reassembles structured fault/event records.
    } else if (header.esp_message_type == BAT_FAULT_EVENTS &&
               len >= static_cast<int>(ESPNOW_HEADER_SIZE + 4U)) {
        const auto* chunk =
            reinterpret_cast<const BATTERY_FAULT_EVENTS_CHUNK_TYPE*>(incoming_data + ESPNOW_HEADER_SIZE);
        const size_t payload_len = static_cast<size_t>(len) - ESPNOW_HEADER_SIZE;
        const size_t available_record_bytes = payload_len - 4U;
        const uint8_t max_records_in_message =
            static_cast<uint8_t>(available_record_bytes / sizeof(BATTERY_FAULT_EVENT_RECORD_TYPE));
        const uint8_t records_in_chunk =
            (chunk->records_in_chunk < max_records_in_message) ? chunk->records_in_chunk : max_records_in_message;
        const uint8_t capped_total_events = (chunk->total_events <= 128U) ? chunk->total_events : 128U;

        if (chunk->transfer_id != shared_state.fault_events_transfer_id) {
            shared_state.fault_events_transfer_id = chunk->transfer_id;
            shared_state.fault_event_count = capped_total_events;
            memset(shared_state.fault_events, 0, sizeof(shared_state.fault_events));
        }

        for (uint8_t i = 0; i < records_in_chunk; ++i) {
            const uint16_t target_index = static_cast<uint16_t>(chunk->start_index) + i;
            if (target_index >= 128U) {
                break;
            }
            shared_state.fault_events[target_index] = chunk->records[i];
        }

        if (chunk->total_events == 0U) {
            shared_state.fault_event_count = 0;
        }

        shared_state.last_fault_events_ms = millis();
        shared_state.has_fault_events = true;
        shared_state.dirty = true;
    // BAT_REMOTE_STATE supplies system, inverter, contactor and pause telemetry.
    } else if (header.esp_message_type == BAT_REMOTE_STATE &&
               len >= static_cast<int>(ESPNOW_HEADER_SIZE + sizeof(BATTERY_REMOTE_STATE_TYPE))) {
        memcpy(&shared_state.remote_state,
               incoming_data + ESPNOW_HEADER_SIZE,
               sizeof(BATTERY_REMOTE_STATE_TYPE));
        shared_state.last_remote_state_ms = millis();
        shared_state.has_remote_state = true;
        clear_pending_command_if_ack_matches(shared_state, shared_state.remote_state);
        shared_state.dirty = true;
    } else {
        shared_state.ignored_packet_count++;
    }
    portEXIT_CRITICAL_ISR(&g_state_mux);
}

// Decode Battery Emulator protocol v2 frames. Unknown keys are skipped by length,
// which keeps this receiver forward-compatible with newer firmware fields.
static uint32_t v2_uint(const uint8_t *p, size_t n) {
    uint32_t v = 0; if (n > 4) n = 4;
    for (size_t i = 0; i < n; ++i) v |= static_cast<uint32_t>(p[i]) << (8U * i);
    return v;
}
static int32_t v2_sint(const uint8_t *p, size_t n) {
    uint32_t v = v2_uint(p, n);
    if (n && n < 4 && (p[n - 1] & 0x80U)) v |= (~0UL) << (8U * n);
    return static_cast<int32_t>(v);
}

#if ESP_IDF_VERSION_MAJOR >= 5
void on_data_recv_v2(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    (void)recv_info;
#else
void on_data_recv_v2(const uint8_t *mac, const uint8_t *data, int len) {
    (void)mac;
#endif
    if (!data || len < static_cast<int>(ESPNOW_V2_HEADER_SIZE) || data[0] != ESPNOW_V2_MAGIC_0 ||
        data[1] != ESPNOW_V2_MAGIC_1 || data[2] != ESPNOW_V2_VERSION) return;
    const uint8_t frame = data[3], battery_id = data[6];
    const int slot = (battery_id >= 1U && battery_id <= BATTERY_SLOT_COUNT) ? static_cast<int>(battery_id - 1U) : 0;
    SharedBatteryState &s = g_shared_states[slot];
    const uint16_t emulator_id = static_cast<uint16_t>(data[4] | (static_cast<uint16_t>(data[5]) << 8));
    const uint32_t now = millis();
    portENTER_CRITICAL_ISR(&g_state_mux);
    s.rx_packet_count++; s.rx_event_pending = true; s.last_message_type = frame;
    s.last_message_len = len; s.emulator_id = emulator_id; s.battery_id = battery_id; s.last_packet_ms = now;
    if (frame == V2_FRAME_BATTERY) {
        // A v2 frame may omit optional values; clear the previous snapshot so omitted
        // keys do not masquerade as current measurements.
        s.info = BATTERY_INFO_TYPE{};
        s.status = BATTERY_STATUS_TYPE{};
        s.has_info = true; s.has_status = true; s.last_info_ms = now; s.last_status_ms = now;
    } else if (frame == V2_FRAME_CELLS) {
        s.has_cells_16bit = true; s.last_cells_16bit_ms = now; s.has_cells = true; s.last_cells_ms = now;
    } else if (frame == V2_FRAME_SYSTEM) {
        // System network fields are omitted when the emulator is not associated;
        // clear the previous SSID/RSSI so the main page cannot show stale data.
        s.remote_net_info = BATTERY_REMOTE_NET_INFO_TYPE{};
        s.has_remote_net_info = false;
    }
    const uint8_t *p = data + ESPNOW_V2_HEADER_SIZE, *end = data + len;
    uint16_t cell_index = 0, cell_count = 0;
    uint8_t event_index = 0, event_total = 0, event_severity = 0xFFU;
    uint8_t event_state = 0xFFU, event_count = 0;
    int16_t event_data = 0;
    char event_name[80] = {0};
    char event_message[192] = {0};
    bool has_event_message = false;
    while (p + 2 <= end) {
        const uint8_t key = p[0], tag = p[1]; p += 2;
        const uint8_t type = tag >> 5, lc = tag & 0x1FU;
        size_t n = lc;
        if (lc == 30U) { if (p + 1 > end) break; n = *p++; }
        else if (lc == 31U) { if (p + 2 > end) break; n = p[0] | (static_cast<size_t>(p[1]) << 8); p += 2; }
        if (p + n > end) break;
        const uint8_t *v = p;
        if (frame == V2_FRAME_BATTERY) {
            switch (key) {
                case V2_NUMBER_OF_CELLS: if (n >= 1) s.info.number_of_cells = v[0]; break;
                case V2_CHEMISTRY: if (n >= 1) s.info.chemistry = v[0]; break;
                case V2_TOTAL_CAPACITY: s.info.total_capacity_Wh = v2_uint(v,n); break;
                case V2_REPORTED_CAPACITY: s.info.reported_total_capacity_Wh = v2_uint(v,n); break;
                case V2_MAX_DESIGN_VOLTAGE: s.info.max_design_voltage_dV = v2_uint(v,n); break;
                case V2_MIN_DESIGN_VOLTAGE: s.info.min_design_voltage_dV = v2_uint(v,n); break;
                case V2_MAX_CELL_DESIGN: s.info.max_cell_voltage_mV = v2_uint(v,n); break;
                case V2_MIN_CELL_DESIGN: s.info.min_cell_voltage_mV = v2_uint(v,n); break;
                case V2_MAX_CELL_DEVIATION: s.info.max_cell_voltage_deviation_mV = v2_uint(v,n); break;
                case V2_SOC: s.status.reported_soc = v2_uint(v,n); break;
                case V2_SOC_REAL: s.status.real_soc = v2_uint(v,n); break;
                case V2_SOH: s.status.soh_pptt = v2_uint(v,n); break;
                case V2_VOLTAGE: s.status.voltage_dV = v2_uint(v,n); break;
                case V2_CURRENT: s.status.current_dA = v2_sint(v,n); break;
                case V2_REPORTED_CURRENT: s.status.reported_current_dA = v2_sint(v,n); break;
                case V2_ACTIVE_POWER: s.status.active_power_W = v2_sint(v,n); break;
                case V2_REMAINING: s.status.remaining_capacity_Wh = v2_uint(v,n); break;
                case V2_REPORTED_REMAINING: s.status.reported_remaining_capacity_Wh = v2_uint(v,n); break;
                case V2_MAX_CHARGE_POWER: s.status.max_charge_power_W = v2_uint(v,n); break;
                case V2_MAX_DISCHARGE_POWER: s.status.max_discharge_power_W = v2_uint(v,n); break;
                case V2_MAX_CHARGE_CURRENT: s.status.max_charge_current_dA = v2_uint(v,n); break;
                case V2_MAX_DISCHARGE_CURRENT: s.status.max_discharge_current_dA = v2_uint(v,n); break;
                case V2_OVERRIDE_CHARGE: s.status.override_charge_power_W = v2_uint(v,n); break;
                case V2_OVERRIDE_DISCHARGE: s.status.override_discharge_power_W = v2_uint(v,n); break;
                case V2_CELL_MAX: s.status.cell_max_voltage_mV = v2_uint(v,n); break;
                case V2_CELL_MIN: s.status.cell_min_voltage_mV = v2_uint(v,n); break;
                case V2_TEMP_MAX: s.status.temperature_max_dC = v2_sint(v,n); break;
                case V2_TEMP_MIN: s.status.temperature_min_dC = v2_sint(v,n); break;
                case V2_TOTAL_CHARGED: s.status.total_charged_battery_Wh = v2_sint(v,n); break;
                case V2_TOTAL_DISCHARGED: s.status.total_discharged_battery_Wh = v2_sint(v,n); break;
                case V2_INSULATION: s.aux_info.isolation_resistance_kohm = v2_uint(v,n); s.has_aux_info = true; s.last_aux_info_ms = now; break;
                case V2_BALANCING: s.status.balancing_status = v2_uint(v,n); break;
                case V2_REAL_BMS: s.status.real_bms_status = v2_uint(v,n); break;
                case V2_CAN_ALIVE: s.status.CAN_battery_still_alive = v2_uint(v,n); break;
                case V2_CAN_ERRORS: s.status.CAN_error_counter = v2_uint(v,n); break;
                case V2_LED_MODE: s.status.led_mode = v2_uint(v,n); break;
                case V2_DETECTED: if (n >= 1 && !v[0]) s.status.CAN_battery_still_alive = 0; break;
                default: break;
            }
        } else if (frame == V2_FRAME_SYSTEM) {
            s.has_remote_state = true; s.last_remote_state_ms = now;
            if (key == V2_PAUSE_STATUS && n >= 1) s.remote_state.pause_status = v[0];
            else if (key == V2_CONTACTORS && n >= 1) s.remote_state.contactors_engaged = v[0];
            else if (key == V2_SYSTEM_STATUS && n >= 1) s.remote_state.system_status = v[0];
            else if (key == V2_INVERTER_ALIVE && n >= 1) {
                // v2 exposes an inverter keepalive countdown rather than the old
                // boolean permission flag; a non-zero countdown means inverter online.
                s.remote_state.inverter_allows_contactor_closing = v[0] != 0;
            }
            else if (key == V2_EQUIPMENT_STOP && n >= 1) s.remote_state.equipment_stop_active = v[0] != 0;
            else if (key == V2_WIFI_RSSI && n >= 1) {
                s.remote_net_info.rssi_dbm = static_cast<int16_t>(v2_sint(v, n));
                s.has_remote_net_info = true;
            } else if (key == V2_IP_ADDRESS && n == 4) {
                snprintf(s.remote_net_info.ip_address, sizeof(s.remote_net_info.ip_address),
                         "%u.%u.%u.%u", v[0], v[1], v[2], v[3]);
                s.has_remote_net_info = true;
            } else if (key == V2_SSID && n < sizeof(s.remote_net_info.ssid)) {
                memcpy(s.remote_net_info.ssid, v, n); s.remote_net_info.ssid[n] = '\0';
                s.has_remote_net_info = true;
            }
        } else if (frame == V2_FRAME_CELLS) {
            if (key == V2_CELL_COUNT) cell_count = static_cast<uint16_t>(v2_uint(v,n));
            else if (key == V2_CELL_INDEX) cell_index = static_cast<uint16_t>(v2_uint(v,n));
            else if (key == V2_CELL_VOLTAGES && type == 6) {
                const size_t count = n / 2U;
                for (size_t i = 0; i < count && cell_index + i < MAX_UI_CHART_POINTS; ++i)
                    s.cells_16bit_mV[cell_index + i] = static_cast<uint16_t>(v[2*i] | (static_cast<uint16_t>(v[2*i+1]) << 8));
                s.cell16_total_cells = static_cast<uint8_t>(cell_count > MAX_UI_CHART_POINTS ? MAX_UI_CHART_POINTS : cell_count);
            }
        } else if (frame == V2_FRAME_EVENT) {
            if (key == V2_EVENT_INDEX && n >= 1) {
                event_index = v[0];
            } else if (key == V2_EVENT_TOTAL && n >= 1) event_total = v[0];
            else if (key == V2_EVENT_SEVERITY && n >= 1) event_severity = v[0];
            else if (key == V2_EVENT_STATE && n >= 1) event_state = v[0];
            else if (key == V2_EVENT_COUNT && n >= 1) event_count = v[0];
            else if (key == V2_EVENT_DATA_I16) event_data = static_cast<int16_t>(v2_sint(v, n));
            else if (key == V2_EVENT_NAME && type == 4) {
                const size_t copy = (n < sizeof(event_name) - 1U) ? n : sizeof(event_name) - 1U;
                memcpy(event_name, v, copy); event_name[copy] = '\0';
            } else if (key == V2_EVENT_MESSAGE && type == 4) {
                const size_t copy = (n < sizeof(event_message) - 1U) ? n : sizeof(event_message) - 1U;
                memcpy(event_message, v, copy); event_message[copy] = '\0';
                has_event_message = true;
            }
        }
        p += n;
    }

    if (frame == V2_FRAME_EVENT && has_event_message) {
        // Build one readable block per event. The list is only rendered after the
        // final frame in the replay batch, preventing scroll resets mid-batch.
        if (event_index == 0U) s.fault_text[0] = '\0';
        size_t used = strlen(s.fault_text);
        if (used < sizeof(s.fault_text) - 1U) {
            const char *separator = used > 0U ? "------------------------\n" : "";
            snprintf(s.fault_text + used,
                     sizeof(s.fault_text) - used,
                     "%s#%u/%u  [%s]\n%s\n%s\nState: %s   Data: %d   Count: %u\n",
                     separator,
                     static_cast<unsigned>(event_index + 1U),
                     static_cast<unsigned>(event_total),
                     fault_level_to_text(event_severity),
                     event_name[0] != '\0' ? event_name : "EVENT",
                     event_message,
                     fault_state_to_text(event_state),
                     static_cast<int>(event_data),
                     static_cast<unsigned>(event_count));
        }
        s.has_fault_text = true;
        s.last_fault_text_ms = now;
    }

    s.dirty = frame != V2_FRAME_EVENT || (data[7] & ESPNOW_V2_FLAG_MORE_CHUNKS) == 0U;
    portEXIT_CRITICAL_ISR(&g_state_mux);
}

}  // namespace

void espnow_receiver_init() {
    // SquareLine no longer exports dedicated version labels on the main pages.

    // Prepare the RGB status LED.
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    set_status_led(0, 0, 255);

    // Receiver works best without Wi-Fi sleep.
    WiFi.setSleep(false);

    // If the screen is on STA Wi-Fi, use its current channel.
    // Otherwise stay on the fixed ESP-NOW channel.
    const int active_channel = screen_network_is_sta_connected() ? screen_network_sta_channel() : ESPNOW_WIFI_CHANNEL;
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(active_channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // Start ESP-NOW stack.
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    // Register a broadcast peer so command send path can use esp_now_send.
    esp_now_peer_info_t peer_info{};
    memcpy(peer_info.peer_addr, g_broadcast_address, sizeof(g_broadcast_address));
    // Use the current Wi-Fi radio channel instead of locking the peer to the
    // channel that happened to be active during init. This keeps command send
    // working after the screen later joins a STA network on another channel.
    peer_info.channel = 0;
    peer_info.encrypt = false;
    if (!esp_now_is_peer_exist(g_broadcast_address)) {
        if (esp_now_add_peer(&peer_info) != ESP_OK) {
            Serial.println("ESP-NOW broadcast peer add failed");
        }
    }

    // Register receive callback after ESP-NOW is ready.
    // Battery Emulator main now emits protocol v2 TLV frames.
    esp_now_register_recv_cb(on_data_recv_v2);
    Serial.printf("ESP-NOW receiver ready on WiFi channel %d\n", active_channel);
    if (screen_network_is_sta_connected() && active_channel != ESPNOW_WIFI_CHANNEL) {
        Serial.printf("ESP-NOW warning: STA uses channel %d while sender expects %u\n",
                      active_channel,
                      static_cast<unsigned>(ESPNOW_WIFI_CHANNEL));
    }
}

// Public helper used by main.cpp right after ui_init().
void espnow_receiver_show_main_screen() {
    load_runtime_main_screen();
}

void espnow_receiver_update() {
    // Rebind runtime callbacks after any SquareLine screen recreation.
    install_navigation_overrides();
    // Fix active screen if single/double battery mode changed.
    enforce_runtime_screen_flow();
    update_main_ip_label();

    // Process both battery slots independently.
    for (uint8_t slot = 0; slot < BATTERY_SLOT_COUNT; ++slot) {
        UiBatteryState snapshot{};
        bool dirty = false;

        // Copy shared callback-owned state into a local snapshot so LVGL work
        // can happen outside the critical section.
        portENTER_CRITICAL(&g_state_mux);
        const SharedBatteryState &shared_state = g_shared_states[slot];
        snapshot.has_info = shared_state.has_info;
        snapshot.has_status = shared_state.has_status;
        snapshot.has_cells = shared_state.has_cells;
        snapshot.has_cells_16bit = shared_state.has_cells_16bit;
        snapshot.has_fault_text = shared_state.has_fault_text;
        snapshot.has_fault_events = shared_state.has_fault_events;
        snapshot.has_aux_info = shared_state.has_aux_info;
        snapshot.has_remote_net_info = shared_state.has_remote_net_info;
        snapshot.has_remote_state = shared_state.has_remote_state;
        snapshot.last_packet_ms = shared_state.last_packet_ms;
        snapshot.last_info_ms = shared_state.last_info_ms;
        snapshot.last_status_ms = shared_state.last_status_ms;
        snapshot.last_cells_ms = shared_state.last_cells_ms;
        snapshot.last_aux_info_ms = shared_state.last_aux_info_ms;
        snapshot.emulator_id = shared_state.emulator_id;
        snapshot.battery_id = shared_state.battery_id;
        snapshot.last_message_type = shared_state.last_message_type;
        snapshot.last_message_len = shared_state.last_message_len;
        snapshot.rx_packet_count = shared_state.rx_packet_count;
        snapshot.ignored_packet_count = shared_state.ignored_packet_count;
        snapshot.rx_event_pending = shared_state.rx_event_pending;
        snapshot.info = shared_state.info;
        snapshot.status = shared_state.status;
        snapshot.aux_info = shared_state.aux_info;
        snapshot.remote_net_info = shared_state.remote_net_info;
        snapshot.cells = shared_state.cells;
        memcpy(snapshot.cells_16bit_mV, shared_state.cells_16bit_mV, sizeof(snapshot.cells_16bit_mV));
        snapshot.last_cells_16bit_ms = shared_state.last_cells_16bit_ms;
        snapshot.cell16_total_cells = shared_state.cell16_total_cells;
        memcpy(snapshot.fault_text, shared_state.fault_text, sizeof(snapshot.fault_text));
        snapshot.last_fault_text_ms = shared_state.last_fault_text_ms;
        memcpy(snapshot.fault_events, shared_state.fault_events, sizeof(snapshot.fault_events));
        snapshot.fault_event_count = shared_state.fault_event_count;
        snapshot.last_fault_events_ms = shared_state.last_fault_events_ms;
        snapshot.remote_state = shared_state.remote_state;
        snapshot.last_remote_state_ms = shared_state.last_remote_state_ms;
        snapshot.last_sent_command_sequence = shared_state.last_sent_command_sequence;
        snapshot.pending_command_sequence = shared_state.pending_command_sequence;
        snapshot.pending_command_id = shared_state.pending_command_id;
        snapshot.pending_command_value = shared_state.pending_command_value;
        snapshot.pending_command_sent_ms = shared_state.pending_command_sent_ms;
        snapshot.command_pending = shared_state.command_pending;
        dirty = shared_state.dirty;
        g_shared_states[slot].dirty = false;
        g_shared_states[slot].rx_event_pending = false;
        portEXIT_CRITICAL(&g_state_mux);

        if (slot == 0U) {
            update_main_network_labels();
            update_main_ip_label();
        }

        // Print RX summary only occasionally to avoid serial spam.
        if (snapshot.rx_event_pending) {
            const uint32_t now = millis();
            if (now - g_last_rx_log_ms >= RX_LOG_INTERVAL_MS) {
                g_last_rx_log_ms = now;
                Serial.printf("ESP-NOW RX #%lu: type=%u len=%d emulator=%u battery=%u ignored=%lu hasInfo=%u hasStatus=%u\n",
                              static_cast<unsigned long>(snapshot.rx_packet_count),
                              static_cast<unsigned>(snapshot.last_message_type),
                              snapshot.last_message_len,
                              static_cast<unsigned>(snapshot.emulator_id),
                              static_cast<unsigned>(snapshot.battery_id),
                              static_cast<unsigned long>(snapshot.ignored_packet_count),
                              snapshot.has_info ? 1U : 0U,
                              snapshot.has_status ? 1U : 0U);
            }
        }

        // Redraw when something changed, or when data became stale.
        if (dirty || !snapshot.has_status || (millis() - snapshot.last_status_ms > PACKET_TIMEOUT_MS)) {
            apply_cached_or_waiting_ui(slot, snapshot);
        }
    }
}
