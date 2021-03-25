#include "field_info.h"
#include "../utils/read_binary.h"
#include "../utils/utils.h"

using namespace std;

field_info field_info::read(ifstream& stream, const struct class_file* a_class) {
    auto result = field_info();
    read_u2(stream, result.access_flags);
    read_u2(stream, result.name_index);
    read_u2(stream, result.descriptor_index);
    read_u2(stream, result.attributes_count);
    for (int i = 0; i < result.attributes_count; i++) {
        result.attributes.push_back(attribute_info::read(stream, a_class));
    }
    return result;
}
