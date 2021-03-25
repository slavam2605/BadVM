#include "attribute_info.h"
#include "code_attribute_info.h"
#include "../utils/read_binary.h"
#include "../utils/constant_pool_utils.h"

using namespace std;

attribute_info attribute_info::read(ifstream& stream, const class_file* a_class) {
    auto result = attribute_info();
    read_u2(stream, result.attribute_name_index);
    read_u4(stream, result.attribute_length);
    auto attribute_name = read_utf8_string(a_class, result.attribute_name_index);
    if (attribute_name == ai_code) {
        result.info = reinterpret_cast<uint8_t*>(code_attribute_info::read(stream, a_class));
    } else {
        result.info = new uint8_t[result.attribute_length];
        stream.read(reinterpret_cast<char*>(result.info), result.attribute_length);
    }
    return result;
}
