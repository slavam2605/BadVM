#ifndef BADJVM_OBJECT_INTRINSICS_H
#define BADJVM_OBJECT_INTRINSICS_H

#include <cstdint>
#include <string>
#include "vm.h"

struct __attribute__((packed)) string_object {
    jvm_reference data;
    jvm_byte coder;
    jvm_int hash;
};

struct __attribute__((packed)) object_header {
    jvm_reference class_reference;
};

struct __attribute__((packed)) array_header {
    jvm_int length;
};

jvm_reference create_interned_string_instance(vm* jvm, const std::string& string_utf8);
jvm_reference create_array_instance(vm* jvm, int32_t length, uint8_t element_size, bool init = true);
jvm_int array_length(jvm_reference reference);

template <class T>
T* array_element(jvm_reference reference, jvm_int index) {
    void* pointer = reference + sizeof(object_header) + sizeof(array_header) + index * sizeof(T);
    return reinterpret_cast<T*>(pointer);
}

#endif //BADJVM_OBJECT_INTRINSICS_H
