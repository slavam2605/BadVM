#include "object_intrinsics.h"
#include <cstring>
#include <codecvt>
#include <locale>
#include "jvm_defines.h"

using namespace std;

static constexpr int8_t string_coder_latin1 = 0;
static constexpr int8_t string_coder_utf16 = 1;

jvm_reference create_interned_string_instance(vm* jvm, const string& string_utf8) {
    static wstring_convert<codecvt_utf8_utf16<char16_t>, char16_t> converter;
    u16string string_utf16 = converter.from_bytes(string_utf8);
    auto intern_iter = jvm->string_table.find(string_utf16);
    if (intern_iter != jvm->string_table.end())
        return intern_iter->second;

    constexpr uint64_t java_object_size = sizeof(string_object);
    jvm_reference string_reference = jvm->heap.allocate(java_object_size, false);
    jvm_reference data_reference = create_array_instance(jvm, string_utf16.length(), sizeof(jvm_char), false);
    memcpy(data_reference + sizeof(object_header) + sizeof(array_header), string_utf16.data(), sizeof(jvm_char) * string_utf16.length());
    auto object = reinterpret_cast<string_object*>(string_reference);
    object->data = data_reference;
    object->coder = string_coder_utf16;
    object->hash = 0;
    jvm->string_table[string_utf16] = string_reference;
    return string_reference;
}

jvm_reference create_array_instance(vm* jvm, int32_t length, uint8_t element_size, bool init) {
    jvm_reference reference = jvm->heap.allocate(sizeof(object_header) + sizeof(array_header) + length * element_size, init);
    reinterpret_cast<array_header*>(reference + sizeof(object_header))->length = length;
    return reference;
}

int32_t array_length(jvm_reference reference) {
    auto header = reinterpret_cast<array_header*>(reference + sizeof(object_header));
    return header->length;
}