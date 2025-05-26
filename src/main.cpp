/////////gitghrtfhef
#include "Arduino.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "string.h"



#define UART_PORT        UART_NUM_1
#define CRSF_PIN         GPIO_NUM_17
#define PACKET_PERIOD_US 3000
#define UART_MODE_UART_HALF_DUPLEX 0x00000003


// Симульовані параметри
uint8_t rate = 2;
uint8_t power = 20;
bool is_bound = false;
uint8_t profile = 0;
uint8_t failsafe = 0;
bool dyn_power = false;

const char* model_name = "ESP32-S3 TX";
const char* fw_version = "v3.3.0";
const char* bind_phrase = "DRONE123";

uint8_t crsf_packet[96];

void build_crsf_rc_channels_packet() {
    uint16_t channels[16];
    for (int i = 0; i < 16; i++) channels[i] = 1500;

    crsf_packet[0] = 0xC8;
    crsf_packet[1] = 0x16;
    crsf_packet[2] = 22;

    uint8_t *payload = &crsf_packet[3];
    uint64_t bits = 0;
    int bitpos = 0;

    for (int i = 0; i < 16; i++) {
        bits |= ((uint64_t)(channels[i] & 0x07FF) << bitpos);
        bitpos += 11;
    }

    for (int i = 0; i < 22; i++) {
        payload[i] = (bits >> (8 * i)) & 0xFF;
    }

    uint8_t crc = 0;
    for (int i = 1; i < 3 + 22; i++) crc ^= crsf_packet[i];
    crsf_packet[3 + 22] = crc;
}

void send_crsf_packet() {
    build_crsf_rc_channels_packet();
    uart_write_bytes(UART_PORT, (const char *)crsf_packet, 3 + 22 + 1);
}

void send_param_response(uint8_t param_id, const char* name, uint8_t val, uint8_t min, uint8_t max) {
    uint8_t len = 4 + strlen(name);
    uint8_t packet[96];
    packet[0] = 0xEE;
    packet[1] = 0x2C;
    packet[2] = len;

    packet[3] = param_id;
    packet[4] = 0x01;  // type: uint8
    packet[5] = min;
    packet[6] = max;
    packet[7] = val;

    strcpy((char*)&packet[8], name);

    uint8_t crc = 0;
    for (int i = 1; i < 3 + len; i++) crc ^= packet[i];
    packet[3 + len] = crc;

    uart_write_bytes(UART_PORT, (const char*)packet, 4 + len);
    Serial.printf("📤 Param %d [%s] = %d\n", param_id, name, val);
}

void send_device_info() {
    const char* name = "ESP32-S3-TX";
    uint8_t name_len = strlen(name);
    uint8_t packet[32] = {
        0xEE, 0x29, (uint8_t)(6 + name_len),
        0x01,           // device type: TX
        0x00, 0x01,     // hw ID
        0x03, 0x03      // fw version
    };
    memcpy(&packet[8], name, name_len);
    uint8_t crc = 0;
    for (int i = 1; i < 8 + name_len; i++) crc ^= packet[i];
    packet[8 + name_len] = crc;
    uart_write_bytes(UART_PORT, (const char*)packet, 9 + name_len);
    Serial.println("📤 Device Info sent");
}

void handle_param_request(uint8_t id) {
    switch (id) {
        case 0: send_param_response(0, "Rate", rate, 0, 4); break;
        case 1: send_param_response(1, "TXPower", power, 10, 250); break;
        case 2: send_param_response(2, "Bind", is_bound ? 1 : 0, 0, 1); break;
        case 3: send_param_response(3, "Profile", profile, 0, 2); break;
        case 4: send_param_response(4, "Failsafe", failsafe, 0, 1); break;
        case 5: send_param_response(5, "DynPower", dyn_power ? 1 : 0, 0, 1); break;
        default: Serial.printf("❔ Unknown param ID %d\n", id); break;
    }
}

void handle_param_write(uint8_t id, uint8_t value) {
    switch (id) {
        case 0: rate = value; break;
        case 1: power = value; break;
        case 2: is_bound = (value == 1); break;
        case 3: profile = value; break;
        case 4: failsafe = value; break;
        case 5: dyn_power = (value == 1); break;
    }
    handle_param_request(id); // echo updated param
}

void IRAM_ATTR timer_callback(void *arg)
{
    send_crsf_packet();
}

void parse_incoming_crsf(const uint8_t *data, size_t len) 
{
    if (len < 5) return;
    uint8_t type = data[1];
    uint8_t length = data[2];
    uint8_t crc = 0;
    for (int i = 1; i < 3 + length; i++) crc ^= data[i];
    if (crc != data[3 + length]) {
        Serial.println("❌ CRC Error");
        return;
    }

    if (type == 0x28) {
        send_device_info();
    } else if (type == 0x2B) {
        handle_param_request(data[3]);
    } else if (type == 0x2D) {
        handle_param_write(data[3], data[4]);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("🚀 ESP32-S3 emulates full ExpressLRS Lua-compatible TX");

    const uart_config_t uart_config = {
        .baud_rate = 420000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_PORT, 512, 512, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, CRSF_PIN, CRSF_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // uart_set_mode(UART_PORT, UART_MODE_UART_HALF_DUPLEX);
    uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX);
    // UART1.conf0.bit_conf0.txd_brk = 1;

    const esp_timer_create_args_t timer_args = 
    {
        .callback = &timer_callback,
        .name = "crsf_tx_timer"
    };

    esp_timer_handle_t timer;
    // esp_timer_create(&timer_args, &timer);
    // esp_timer_start_periodic(timer, PACKET_PERIOD_US);
}


void loop() {
    static unsigned long lastSendTime = 0;
    const unsigned long intervalMicros = 3000; // 3 мс = ~333 Гц

    uint32_t now = micros();

    // Надсилати CRSF-пакет кожні 3 мс
    if (now - lastSendTime >= intervalMicros) {
        send_crsf_packet();
        lastSendTime = now;
    }

    // Прийом UART даних від модуля
    uint8_t buf[96];
    int len = uart_read_bytes(UART_PORT, buf, sizeof(buf), 20 / portTICK_PERIOD_MS);
    if (len > 0) {
        parse_incoming_crsf(buf, len);
    }
}
