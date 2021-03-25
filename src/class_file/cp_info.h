#ifndef BADJVM_CP_INFO_H
#define BADJVM_CP_INFO_H

#include <fstream>

enum class constant_pool_tag : uint8_t {
    CONSTANT_Class = 7,
    CONSTANT_Fieldref = 9,
    CONSTANT_Methodref = 10,
    CONSTANT_InterfaceMethodref = 11,
    CONSTANT_String = 8,
    CONSTANT_Integer = 3,
    CONSTANT_Float = 4,
    CONSTANT_Long = 5,
    CONSTANT_Double = 6,
    CONSTANT_NameAndType = 12,
    CONSTANT_Utf8 = 1,
    CONSTANT_MethodHandle = 15,
    CONSTANT_MethodType = 16,
    CONSTANT_InvokeDynamic = 18,
};

struct cp_info {
    constant_pool_tag tag;
    uint8_t* info;

    static cp_info read(std::ifstream& stream);
};

struct cp_class_info {
    uint16_t name_index;
};

// the same struct for Fieldref, Methodref, and InterfaceMethodref
struct cp_fmi_ref_info {
    uint16_t class_index;
    uint16_t name_and_type_index;
};

struct cp_name_and_type_info {
    uint16_t name_index;
    uint16_t descriptor_index;
};

struct cp_string {
    uint16_t string_index;
};

struct cp_utf8_info {
    uint16_t length;
    uint8_t* bytes;
};

#endif //BADJVM_CP_INFO_H
