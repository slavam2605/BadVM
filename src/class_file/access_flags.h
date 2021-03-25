#ifndef BADJVM_ACCESS_FLAGS_H
#define BADJVM_ACCESS_FLAGS_H

#include <cstdint>

static constexpr uint16_t ACC_PUBLIC = 0x0001;
static constexpr uint16_t ACC_PRIVATE = 0x0002;
static constexpr uint16_t ACC_PROTECTED = 0x0004;
static constexpr uint16_t ACC_STATIC = 0x0008;
static constexpr uint16_t ACC_FINAL = 0x0010;
static constexpr uint16_t ACC_SUPER = 0x0020;
static constexpr uint16_t ACC_SYNCHRONIZED = 0x0020;
static constexpr uint16_t ACC_VOLATILE = 0x0040;
static constexpr uint16_t ACC_BRIDGE = 0x0040;
static constexpr uint16_t ACC_TRANSIENT = 0x0080;
static constexpr uint16_t ACC_VARARGS = 0x0080;
static constexpr uint16_t ACC_NATIVE = 0x0100;
static constexpr uint16_t ACC_INTERFACE = 0x0200;
static constexpr uint16_t ACC_ABSTRACT = 0x0400;
static constexpr uint16_t ACC_STRICT = 0x0800;
static constexpr uint16_t ACC_SYNTHETIC = 0x1000;
static constexpr uint16_t ACC_ANNOTATION = 0x2000;
static constexpr uint16_t ACC_ENUM = 0x4000;
static constexpr uint16_t ACC_MANDATED = 0x8000;

#endif //BADJVM_ACCESS_FLAGS_H
