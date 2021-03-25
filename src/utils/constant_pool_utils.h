#ifndef BADJVM_CONSTANT_POOL_UTILS_H
#define BADJVM_CONSTANT_POOL_UTILS_H

#include "../class_file/class_file.h"
#include <vector>

std::string read_utf8_string(const class_file* class_file, int index);
double read_double(const class_file* class_file, int index);
int read_int(const class_file* class_file, int index);
const cp_fmi_ref_info* read_method_ref(const class_file* class_file, int index);
const cp_fmi_ref_info* read_field_ref(const class_file* class_file, int index);
const cp_class_info* read_class_info(const class_file* class_file, int index);
const cp_name_and_type_info* read_name_and_type(const class_file* class_file, int index);

#endif //BADJVM_CONSTANT_POOL_UTILS_H
