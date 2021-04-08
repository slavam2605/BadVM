#ifndef BADJVM_JIT_COMPILER_H
#define BADJVM_JIT_COMPILER_H

#include <unordered_set>
#include "code_manager.h"
#include "ir_instructions.h"
#include "../class_file/class_file.h"
#include "../class_file/code_attribute_info.h"

struct jit_compiler {
    code_manager manager;

    const void* compile(const class_file* current_class, const method_info& method, const code_attribute_info* code_info);
};

struct jit_local_state {
    std::vector<ir_variable_type> locals;
    std::vector<ir_variable_type> stack;
    int max_stack_size;

    jit_local_state(int locals_count, int max_stack_size);
    ir_variable allocate_stack(ir_variable_type type);
    ir_variable pop_stack(ir_variable_type expected_type);
    ir_variable get_local_read(int index, ir_variable_type expected_type);
    ir_variable get_local_write(int index, ir_variable_type new_type);
};

#endif //BADJVM_JIT_COMPILER_H
