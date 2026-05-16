#include "AP_ThrustSensor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern const AP_HAL::HAL& hal;

void AP_ThrustSensor::init(AP_HAL::UARTDriver *uart)
{
    _uart = uart;
    if (_uart == nullptr) {
        return;
    }
    _uart->begin(9600, 128, 128);
}

void AP_ThrustSensor::update()
{
    if (_uart == nullptr) {
        return;
    }
    send_query_if_due();
    read_and_parse();
}

void AP_ThrustSensor::send_query_if_due()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_query_ms < 50) { // 20Hz
        return;
    }
    _last_query_ms = now;
    const char *cmd = "AT+WEI\r\n";
    _uart->write((const uint8_t*)cmd, strlen(cmd));
}

void AP_ThrustSensor::read_and_parse()
{
    while (_uart->available() > 0) {
        const int16_t c = _uart->read();
        if (c < 0) {
            break;
        }
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            _buf[_idx] = '\0';
            if (_idx > 0) {
                parse_line(_buf);
            }
            _idx = 0;
            continue;
        }
        if (_idx < BUF_LEN - 1) {
            _buf[_idx++] = (char)c;
        } else {
            _idx = 0; // overflow reset
        }
    }
}

void AP_ThrustSensor::parse_line(const char *line)
{
    // expected: ST00123.42,1,0
    if (strncmp(line, "ST", 2) != 0) {
        return;
    }

    const char *p1 = strchr(line, ',');
    if (p1 == nullptr) return;
    const char *p2 = strchr(p1 + 1, ',');
    if (p2 == nullptr) return;

    char weight_str[16]{};
    const size_t wlen = (size_t)(p1 - (line + 2));
    if (wlen == 0 || wlen >= sizeof(weight_str)) return;
    memcpy(weight_str, line + 2, wlen);
    weight_str[wlen] = '\0';

    const float w = strtof(weight_str, nullptr);
    const int s = atoi(p1 + 1);
    const int o = atoi(p2 + 1);

    _weight = w;
    _stable = (uint8_t)(s ? 1 : 0);
    _overload = (uint8_t)(o ? 1 : 0);
    _healthy = true;
}
