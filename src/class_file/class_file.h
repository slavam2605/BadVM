#ifndef BADJVM_CLASS_FILE_H
#define BADJVM_CLASS_FILE_H

#include <string>
#include <vector>
#include <cstdint>
#include <unordered_map>
#include "cp_info.h"
#include "field_info.h"
#include "method_info.h"
#include "attribute_info.h"
#include "../vm/jvm_defines.h"

struct class_file {
    uint16_t access_flags;
    uint16_t this_class;
    uint16_t super_class;
    std::vector<cp_info> constant_pool;
    std::vector<uint16_t> interfaces;
    std::vector<struct field_info> fields;
    std::vector<struct method_info> methods;
    std::vector<struct attribute_info> attributes;

    std::string class_name; // TODO maybe remove
    uint64_t total_object_size;
    std::unordered_map<std::string, uint64_t> field_offsets;
    std::unordered_map<std::string, jvm_reference> static_fields; // references to metaspace

    static class_file* read(struct vm* jvm, const std::string& class_name);
};


#endif //BADJVM_CLASS_FILE_H
