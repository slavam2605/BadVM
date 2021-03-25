#include "heap_manager.h"
#include <cstring>
#include <stdexcept>

heap_manager::heap_manager() {
    heap = new uint8_t[heap_size];
    metaspace = new uint8_t[metaspace_size];
}

jvm_reference heap_manager::null_ref() {
    return nullptr;
}

bool heap_manager::is_null(const jvm_reference& reference) {
    return reference == null_ref();
}

void init_memory(uint8_t* ptr, uint64_t size) {
    // Initialize all fields to default values
    memset(ptr, 0, size);
}

jvm_reference heap_manager::allocate(uint64_t size, bool init) {
    if (offset + size > heap_size)
        throw std::runtime_error("Out of memory: heap");

    if (init) {
        init_memory(heap + offset, size);
    }
    uint64_t object_offset = offset;
    offset += size;
    return heap + object_offset;
}

jvm_reference heap_manager::allocate_metaspace(uint64_t size) {
    if (metaspace_offset + size > metaspace_size)
        throw std::runtime_error("Out of memory: metaspace");

    init_memory(metaspace + metaspace_offset, size);
    uint64_t object_offset = metaspace_offset;
    metaspace_offset += size;
    return metaspace + object_offset;
}
