#include "jvm_value.h"
#include <stdexcept>

using namespace std;

jvm_value jvm_value::as_reference(jvm_reference value) {
    jvm_value result{};
    result.reference_value = value;
    return result;
}

//jvm_value jvm_value::as_return_address(uint64_t value) {
//    jvm_value result{};
//    result.return_address_value = value;
//    return result;
//}

jvm_value jvm_value::as_long(jvm_long value) {
    jvm_value result{};
    result.long_value = value;
    return result;
}

jvm_value jvm_value::as_int(jvm_int value) {
    jvm_value result{};
    result.int_value = value;
    return result;
}

jvm_value jvm_value::as_short(jvm_short value) {
    jvm_value result{};
    result.short_value = value;
    return result;
}

jvm_value jvm_value::as_char(jvm_char value) {
    jvm_value result{};
    result.char_value = value;
    return result;
}

jvm_value jvm_value::as_byte(jvm_byte value) {
    jvm_value result{};
    result.byte_value = value;
    return result;
}

jvm_value jvm_value::as_float(jvm_float value) {
    jvm_value result{};
    result.float_value = value;
    return result;
}

jvm_value jvm_value::as_double(jvm_double value) {
    jvm_value result{};
    result.double_value = value;
    return result;
}

jvm_value jvm_value::as_boolean(jvm_boolean value) {
    jvm_value result{};
    result.boolean_value = value;
    return result;
}

uint8_t atype_size(atype type) {
    switch (type) {
        case atype::T_BOOLEAN: return sizeof(jvm_boolean);
        case atype::T_CHAR: return sizeof(jvm_char);
        case atype::T_FLOAT: return sizeof(jvm_float);
        case atype::T_DOUBLE: return sizeof(jvm_double);
        case atype::T_BYTE: return sizeof(jvm_byte);
        case atype::T_SHORT: return sizeof(jvm_short);
        case atype::T_INT: return sizeof(jvm_int);
        case atype::T_LONG: return sizeof(jvm_long);
        default: throw runtime_error("Unknown atype");
    }
}