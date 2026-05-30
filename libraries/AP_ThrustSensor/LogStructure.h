#pragma once
#include <AP_Logger/LogStructure.h>

struct PACKED log_THST {
    LOG_PACKET_HEADER;
    uint64_t TimeUS;
    float Kg;
    float N;
    uint8_t Stb;
    uint8_t Ovr;
    uint8_t Hlt;
};