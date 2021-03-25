#ifndef BADJVM_CODE_ATTRIBUTE_INFO_H
#define BADJVM_CODE_ATTRIBUTE_INFO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include "attribute_info.h"

struct exception_table_entry {
    uint16_t start_pc;
    uint16_t end_pc;
    uint16_t handler_pc;
    uint16_t catch_type;
};

struct code_attribute_info {
    uint16_t max_stack;
    uint16_t max_locals;
    uint32_t code_length;
    uint8_t* code;
    uint16_t exception_table_length;
    std::vector<exception_table_entry> exception_table;
    uint16_t attributes_count;
    std::vector<attribute_info> attributes;

    static code_attribute_info* read(std::ifstream& stream, const class_file* class_file);
};


#endif //BADJVM_CODE_ATTRIBUTE_INFO_H
