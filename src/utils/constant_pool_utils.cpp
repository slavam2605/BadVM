#include "constant_pool_utils.h"
#include "utils.h"
#include <string>

using namespace std;

std::string read_utf8_string(const class_file* class_file, int index) {
    auto& entry = class_file->constant_pool[index - 1];
    assert(entry.tag == constant_pool_tag::CONSTANT_Utf8)
    auto info = reinterpret_cast<cp_utf8_info*>(entry.info);
    return std::string(reinterpret_cast<char*>(info->bytes), info->length);
}

double read_double(const class_file* class_file, int index) {
    auto& entry = class_file->constant_pool[index - 1];
    assert(entry.tag == constant_pool_tag::CONSTANT_Double)
    auto info = reinterpret_cast<double*>(entry.info);
    return *info;
}

int read_int(const class_file* class_file, int index) {
    auto& entry = class_file->constant_pool[index - 1];
    assert(entry.tag == constant_pool_tag::CONSTANT_Integer)
    auto info = reinterpret_cast<int*>(entry.info);
    return *info;
}

const cp_fmi_ref_info* read_method_ref(const class_file* class_file, int index) {
    auto& entry = class_file->constant_pool[index - 1];
    assert(entry.tag == constant_pool_tag::CONSTANT_Methodref)
    auto info = reinterpret_cast<cp_fmi_ref_info*>(entry.info);
    return info;
}

const cp_fmi_ref_info* read_field_ref(const class_file* class_file, int index) {
    auto& entry = class_file->constant_pool[index - 1];
    assert(entry.tag == constant_pool_tag::CONSTANT_Fieldref)
    auto info = reinterpret_cast<cp_fmi_ref_info*>(entry.info);
    return info;
}

const cp_class_info* read_class_info(const class_file* class_file, int index) {
    auto& entry = class_file->constant_pool[index - 1];
    assert(entry.tag == constant_pool_tag::CONSTANT_Class)
    auto info = reinterpret_cast<cp_class_info*>(entry.info);
    return info;
}

const cp_name_and_type_info* read_name_and_type(const class_file* class_file, int index) {
    auto& entry = class_file->constant_pool[index - 1];
    assert(entry.tag == constant_pool_tag::CONSTANT_NameAndType)
    auto info = reinterpret_cast<cp_name_and_type_info*>(entry.info);
    return info;
}