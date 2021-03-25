#include "code_attribute_info.h"
#include "../utils/read_binary.h"

using namespace std;

code_attribute_info* code_attribute_info::read(ifstream& stream, const class_file* class_file) {
    auto result = new code_attribute_info();
    read_u2(stream, result->max_stack);
    read_u2(stream, result->max_locals);
    read_u4(stream, result->code_length);
    result->code = new uint8_t[result->code_length];
    stream.read(reinterpret_cast<char*>(result->code), result->code_length);
    read_u2(stream, result->exception_table_length);
    for (int i = 0; i < result->exception_table_length; i++) {
        auto entry = exception_table_entry();
        read_u2(stream, entry.start_pc);
        read_u2(stream, entry.end_pc);
        read_u2(stream, entry.handler_pc);
        read_u2(stream, entry.catch_type);
        result->exception_table.push_back(entry);
    }
    read_u2(stream, result->attributes_count);
    for (int i = 0; i < result->attributes_count; i++) {
        result->attributes.push_back(attribute_info::read(stream, class_file));
    }
    return result;
}
