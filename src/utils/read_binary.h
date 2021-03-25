#ifndef BADJVM_READ_BINARY_H
#define BADJVM_READ_BINARY_H

#include <fstream>

void read_u8(std::ifstream& stream, uint64_t& value);
void read_u4(std::ifstream& stream, uint32_t& value);
void read_u2(std::ifstream& stream, uint16_t& value);
void read_u1(std::ifstream& stream, uint8_t& value);

uint64_t read_u8(std::ifstream& stream);
uint32_t read_u4(std::ifstream& stream);
uint16_t read_u2(std::ifstream& stream);
uint8_t read_u1(std::ifstream& stream);

#endif //BADJVM_READ_BINARY_H
