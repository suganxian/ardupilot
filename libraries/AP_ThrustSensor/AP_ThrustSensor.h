#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_ThrustSensor {
public:
    void init(AP_HAL::UARTDriver *uart);
    void update();

    bool healthy() const { return _healthy; }
    float weight() const { return _weight; }
    uint8_t stable() const { return _stable; }
    uint8_t overload() const { return _overload; }

private:
    void send_query_if_due();
    void read_and_parse();
    void parse_line(const char *line);

    AP_HAL::UARTDriver *_uart = nullptr;
    uint32_t _last_query_ms = 0;
    bool _healthy = false;

    float _weight = 0.0f;
    uint8_t _stable = 0;
    uint8_t _overload = 0;

    static constexpr uint16_t BUF_LEN = 64;
    char _buf[BUF_LEN]{};
    uint16_t _idx = 0;
};
