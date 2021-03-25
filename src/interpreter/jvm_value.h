#ifndef BADJVM_JVM_VALUE_H
#define BADJVM_JVM_VALUE_H

#include <cstdint>
#include "../vm/jvm_defines.h"

union jvm_value {
    jvm_reference reference_value;
//    uint64_t return_address_value;
    jvm_long long_value;
    jvm_int int_value;
    jvm_short short_value;
    jvm_char char_value;
    jvm_byte byte_value;
    jvm_float float_value;
    jvm_double double_value;
    jvm_boolean boolean_value;

    static jvm_value as_reference(jvm_reference value);
//    static jvm_value as_return_address(uint64_t value);
    static jvm_value as_long(jvm_long value);
    static jvm_value as_int(jvm_int value);
    static jvm_value as_short(jvm_short value);
    static jvm_value as_char(jvm_char value);
    static jvm_value as_byte(jvm_byte value);
    static jvm_value as_float(jvm_float value);
    static jvm_value as_double(jvm_double value);
    static jvm_value as_boolean(jvm_boolean value);
};

enum class atype : uint8_t {
    T_BOOLEAN = 4,
    T_CHAR = 5,
    T_FLOAT = 6,
    T_DOUBLE = 7,
    T_BYTE = 8,
    T_SHORT = 9,
    T_INT = 10,
    T_LONG = 11
};

uint8_t atype_size(atype type);

#endif //BADJVM_JVM_VALUE_H
