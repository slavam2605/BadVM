#include "cp_info.h"
#include "../utils/read_binary.h"
#include "../utils/utils.h"
#include "../vm/jvm_defines.h"
#include <iostream>

using namespace std;

cp_info cp_info::read(ifstream& stream) {
    auto result = cp_info();
    result.tag = static_cast<constant_pool_tag>(read_u1(stream));
    switch (result.tag) {
        case constant_pool_tag::CONSTANT_Class: {
            auto info = new cp_class_info();
            read_u2(stream, info->name_index);
            result.info = reinterpret_cast<uint8_t*>(info);
            break;
        }
        case constant_pool_tag::CONSTANT_Fieldref:
        case constant_pool_tag::CONSTANT_Methodref:
        case constant_pool_tag::CONSTANT_InterfaceMethodref: {
            auto info = new cp_fmi_ref_info();
            read_u2(stream, info->class_index);
            read_u2(stream, info->name_and_type_index);
            result.info = reinterpret_cast<uint8_t*>(info);
            break;
        }
        case constant_pool_tag::CONSTANT_String: {
            auto info = new cp_string();
            read_u2(stream, info->string_index);
            result.info = reinterpret_cast<uint8_t*>(info);
            break;
        }
        case constant_pool_tag::CONSTANT_Integer: // reading the same bytes
        case constant_pool_tag::CONSTANT_Float: {
            uint32_t bytes = read_u4(stream);
            result.info = reinterpret_cast<uint8_t*>(new uint32_t(bytes));
            break;
        }
        case constant_pool_tag::CONSTANT_Long: // reading the same bytes
        case constant_pool_tag::CONSTANT_Double: {
            uint32_t high_bytes, low_bytes;
            read_u4(stream, high_bytes);
            read_u4(stream, low_bytes);
            uint64_t raw_value = ((uint64_t) high_bytes << 32) + low_bytes;
            result.info = reinterpret_cast<uint8_t*>(new uint64_t(raw_value));
            break;
        }
        case constant_pool_tag::CONSTANT_NameAndType: {
            auto info = new cp_name_and_type_info();
            read_u2(stream, info->name_index);
            read_u2(stream, info->descriptor_index);
            result.info = reinterpret_cast<uint8_t*>(info);
            break;
        }
        case constant_pool_tag::CONSTANT_Utf8: {
            auto info = new cp_utf8_info();
            read_u2(stream, info->length);
            info->bytes = new uint8_t[info->length];
            stream.read(reinterpret_cast<char*>(info->bytes), info->length);
            result.info = reinterpret_cast<uint8_t*>(info);
            break;
        }
        case constant_pool_tag::CONSTANT_MethodHandle:
            assert(false)
        case constant_pool_tag::CONSTANT_MethodType:
            assert(false)
        case constant_pool_tag::CONSTANT_InvokeDynamic:
            assert(false)
        default:
            assert(false)
    }
    return result;
}
