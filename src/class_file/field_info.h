#ifndef BADJVM_FIELD_INFO_H
#define BADJVM_FIELD_INFO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include "attribute_info.h"

struct field_info {
    uint16_t access_flags;
    uint16_t name_index;
    uint16_t descriptor_index;
    uint16_t attributes_count;
    std::vector<struct attribute_info> attributes;

    static field_info read(std::ifstream& stream, const struct class_file* a_class);
};


#endif //BADJVM_FIELD_INFO_H
