#ifndef BADJVM_JVM_DEFINES_H
#define BADJVM_JVM_DEFINES_H

#include <cstdint>

using jvm_reference = uint8_t*;
using jvm_boolean = bool;
using jvm_byte = int8_t;
using jvm_short = int16_t;
using jvm_char = uint16_t;
using jvm_int = int32_t;
using jvm_long = int64_t;
using jvm_float = float;
using jvm_double = double;

#endif //BADJVM_JVM_DEFINES_H
