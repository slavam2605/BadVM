#include <stdexcept>
#include "method_descriptor.h"
#include "../vm/jvm_defines.h"

using namespace std;

method_descriptor method_descriptor::parse(const string& source) {
    method_descriptor result;
    int pos = 0;
    if (source[pos] == '(') {
        pos++;
        while (source[pos] != ')') {
            result.parameters.push_back(field_descriptor::parse(source, pos));
        }
        pos++;
    }
    if (source[pos] == 'V') {
        result.return_void = true;
    } else {
        result.return_descriptor = field_descriptor::parse(source, pos);
    }
    return result;
}

field_descriptor field_descriptor::parse(const string& source, int& pos) {
    field_descriptor result;
    if (source[pos] == 'B' || source[pos] == 'C' || source[pos] == 'D' || source[pos] == 'F' ||
        source[pos] == 'I' || source[pos] == 'J' || source[pos] == 'S' || source[pos] == 'Z') {
        result.base_type = source[pos];
        pos++;
        return result;
    }

    if (source[pos] == 'L') {
        pos++;
        while (source[pos] != ';') {
            result.object_type.push_back(source[pos]);
            pos++;
        }
        pos++;
        return result;
    }
    
    if (source[pos] == '[') {
        while (source[pos] == '[') {
            result.array_dimensions++;
            pos++;
        }
        result.array_type = new field_descriptor(field_descriptor::parse(source, pos));
        return result;
    }
    
    throw runtime_error("Failed to parse a field type: unexpected first char '" + string(1, source[pos]) + "'");
}

base_type_descriptor field_descriptor::get_base_type() const {
    switch (base_type) {
        case 'B': return base_type_descriptor::byte_d;
        case 'C': return base_type_descriptor::char_d;
        case 'D': return base_type_descriptor::double_d;
        case 'F': return base_type_descriptor::float_d;
        case 'I': return base_type_descriptor::int_d;
        case 'J': return base_type_descriptor::long_d;
        case 'S': return base_type_descriptor::short_d;
        case 'Z': return base_type_descriptor::boolean_d;
        default: return base_type_descriptor::reference_d;
    }
}

field_descriptor field_descriptor::parse(const string& source) {
    int pos = 0;
    return parse(source, pos);
}

uint8_t field_descriptor::get_field_size() const {
    switch (base_type) {
        case 'B': return sizeof(jvm_byte);
        case 'C': return sizeof(jvm_char);
        case 'D': return sizeof(jvm_double);
        case 'F': return sizeof(jvm_float);
        case 'I': return sizeof(jvm_int);
        case 'J': return sizeof(jvm_long);
        case 'S': return sizeof(jvm_short);
        case 'Z': return sizeof(jvm_boolean);
        default: return sizeof(jvm_reference);
    }
}
