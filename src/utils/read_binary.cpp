#include "read_binary.h"

using namespace std;

void read_u8(ifstream& stream, uint64_t& value) {
    stream.read(reinterpret_cast<char*>(&value), 8);
    value = __builtin_bswap64(value);
}

void read_u4(ifstream& stream, uint32_t& value) {
    stream.read(reinterpret_cast<char*>(&value), 4);
    value = __builtin_bswap32(value);
}

void read_u2(ifstream& stream, uint16_t& value) {
    stream.read(reinterpret_cast<char*>(&value), 2);
    value = __builtin_bswap16(value);
}

void read_u1(ifstream& stream, uint8_t& value) {
    stream.read(reinterpret_cast<char*>(&value), 1);
}

uint64_t read_u8(ifstream& stream) {
    uint64_t value;
    read_u8(stream, value);
    return value;
}

uint32_t read_u4(ifstream& stream) {
    uint32_t value;
    read_u4(stream, value);
    return value;
}

uint16_t read_u2(ifstream& stream) {
    uint16_t value;
    read_u2(stream, value);
    return value;
}

uint8_t read_u1(ifstream& stream) {
    uint8_t value;
    read_u1(stream, value);
    return value;
}
