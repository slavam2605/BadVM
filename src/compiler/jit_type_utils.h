#ifndef BADJVM_JIT_TYPE_UTILS_H
#define BADJVM_JIT_TYPE_UTILS_H

template <class T>
bool is_int() { return false; }

template <>
bool is_int<jvm_boolean>() { return true; }

template <>
bool is_int<jvm_byte>() { return true; }

template <>
bool is_int<jvm_char>() { return true; }

template <>
bool is_int<jvm_short>() { return true; }

template <>
bool is_int<jvm_int>() { return true; }

template <>
bool is_int<jvm_long>() { return true; }

#endif //BADJVM_JIT_TYPE_UTILS_H
