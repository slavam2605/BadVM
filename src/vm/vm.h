#ifndef BADJVM_VM_H
#define BADJVM_VM_H

#include <string>
#include <cstdint>
#include <unordered_map>
#include <mutex>
#include "../interpreter/jvm_value.h"
#include "../class_file/class_file.h"
#include "../class_file/code_attribute_info.h"
#include "heap_manager.h"
#include "../interpreter/method_descriptor.h"

struct frame_info {
    uint16_t max_stack;
    uint16_t max_locals;
};

struct vm {
    heap_manager heap;
    std::unordered_map<std::string, class_file*> loaded_classes;
    std::unordered_map<std::u16string, jvm_reference> string_table;
    std::unordered_map<void*, std::recursive_mutex> monitor_map;

    bool is_java_lang_object_init(const class_file* current_class, const cp_fmi_ref_info* method_info);
    void invoke_with_parameters(std::vector<jvm_value>& stack, const method_descriptor& descriptor, const class_file* class_info, const code_attribute_info* method_code_info);
    std::ifstream open_class_file(const std::string& class_name);
    std::vector<class_file*> load_super_classes_and_self(class_file* starting_class);
public:
    void start(const std::string& class_name);
    class_file* get_or_load_class(const std::string& name);
    class_file* get_or_load_class(const class_file* current_class, const cp_class_info* class_info);
    std::pair<uint64_t, uint8_t> get_field_info(const class_file* current_class, uint16_t index);
    std::pair<jvm_reference, uint8_t> get_static_field_info(const class_file* current_class, uint16_t index);
    std::pair<const class_file*, const code_attribute_info*> get_code_info(const class_file* current_class, const cp_fmi_ref_info* method_ref);
    std::pair<const class_file*, const code_attribute_info*> get_code_info(class_file* target_class, const std::string& target_method_name, const std::string& target_method_descriptor);
    jvm_value interpret(const class_file* current_class, const code_attribute_info* code_info, const std::vector<jvm_value>&& parameters);
};


#endif //BADJVM_VM_H
