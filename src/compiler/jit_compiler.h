#ifndef BADJVM_JIT_COMPILER_H
#define BADJVM_JIT_COMPILER_H

#include <unordered_set>
#include "code_manager.h"
#include "../class_file/class_file.h"
#include "../class_file/code_attribute_info.h"
#include "code_builder.h"

struct jit_compiler {
    code_manager manager;

    const void* compile(const class_file* current_class, const method_info& method, const code_attribute_info* code_info);
};


#endif //BADJVM_JIT_COMPILER_H
