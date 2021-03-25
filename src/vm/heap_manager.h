#ifndef BADJVM_HEAP_MANAGER_H
#define BADJVM_HEAP_MANAGER_H

#include "jvm_defines.h"
#include <cstdint>

struct heap_manager {
    uint64_t heap_size = 10000;
    uint64_t offset = 0;
    uint8_t* heap;

    uint64_t metaspace_size = 10000;
    uint64_t metaspace_offset = 0;
    uint8_t* metaspace;

public:
    heap_manager();
    jvm_reference allocate(uint64_t size, bool init = true);
    jvm_reference allocate_metaspace(uint64_t size);

    static bool is_null(const jvm_reference& reference);
    static jvm_reference null_ref();
};


#endif //BADJVM_HEAP_MANAGER_H
