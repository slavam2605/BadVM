#include "class_file.h"
#include "../utils/utils.h"
#include "../utils/read_binary.h"
#include "../utils/constant_pool_utils.h"
#include "../interpreter/method_descriptor.h"
#include "../vm/vm.h"
#include "access_flags.h"
#include <iostream>
#include <string>

using namespace std;

void class_file_init_processing(vm* jvm, class_file* a_class) {
    uint64_t object_size = 0;

    auto super_classes = jvm->load_super_classes_and_self(a_class);
    for (auto it = super_classes.rbegin(); it != super_classes.rend(); ++it) {
        auto some_class = *it;
        for (const auto& field : some_class->fields) {
            auto descriptor_string = read_utf8_string(some_class, field.descriptor_index);
            auto descriptor = field_descriptor::parse(descriptor_string);
            auto field_name = read_utf8_string(some_class, field.name_index);
            if ((field.access_flags & ACC_STATIC) != 0) {
                if (some_class != a_class) {
                    // Get already allocated memory from an ancestor class
                    a_class->static_fields[field_name] = some_class->static_fields[field_name];
                } else {
                    // Allocate memory for the static field
                    auto field_reference = jvm->heap.allocate_metaspace(descriptor.get_field_size());
                    a_class->static_fields[field_name] = field_reference;
                }
            } else {
                a_class->field_offsets[field_name] = object_size;
                object_size += descriptor.get_field_size();
            }
        }
    }

    a_class->total_object_size = object_size;
}

class_file* class_file::read(vm* jvm, const string &class_name) {
    auto result = new class_file();
    result->class_name = class_name;
    ifstream stream = jvm->open_class_file(class_name);
    uint32_t magic = read_u4(stream);
    assert(magic == 0xCAFEBABE);
    /*uint32_t minor_version = */read_u2(stream);
    /*uint32_t major_version = */read_u2(stream);
    uint16_t constant_pool_count = read_u2(stream);
    for (int i = 0; i < constant_pool_count - 1; i++) {
        const cp_info& info = cp_info::read(stream);
        result->constant_pool.push_back(info);
        if (info.tag == constant_pool_tag::CONSTANT_Double || info.tag == constant_pool_tag::CONSTANT_Long) {
            // §4.4.5 - CONSTANT_Double and CONSTANT_Long must take two entries in a constant pool
            result->constant_pool.push_back(cp_info());
            i++;
        }
    }
    read_u2(stream, result->access_flags);
    read_u2(stream, result->this_class);
    read_u2(stream, result->super_class);
    uint16_t interfaces_count = read_u2(stream);
    for (int i = 0; i < interfaces_count; i++) {
        result->interfaces.push_back(read_u2(stream));
    }
    uint16_t fields_count = read_u2(stream);
    for (int i = 0; i < fields_count; i++) {
        result->fields.push_back(field_info::read(stream, result));
    }
    uint16_t methods_count = read_u2(stream);
    for (int i = 0; i < methods_count; i++) {
        result->methods.push_back(method_info::read(stream, result));
    }
    uint16_t attributes_count = read_u2(stream);
    for (int i = 0; i < attributes_count; i++) {
        result->attributes.push_back(attribute_info::read(stream, result));
    }
    class_file_init_processing(jvm, result);
    return result;
}