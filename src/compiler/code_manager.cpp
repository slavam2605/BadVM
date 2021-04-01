#include "code_manager.h"
#include <windows.h>
#include <cstring>

code_manager::code_manager() {
    auto buffer = VirtualAlloc(nullptr, code_cache_size, MEM_COMMIT, PAGE_EXECUTE_READ);
    code_cache = reinterpret_cast<uint8_t*>(buffer);
}

code_manager::~code_manager() {
    VirtualFree(code_cache, 0, MEM_RELEASE);
}

uint8_t* code_manager::add_code_chunk(const std::vector<uint8_t>& code) {
    return add_code_chunk(code.data(), code.size());
}

uint8_t* code_manager::add_code_chunk(const void* src, uint64_t size) {
    auto pointer = code_cache + offset;
    DWORD old_protection;
    VirtualProtect(pointer, size, PAGE_READWRITE, &old_protection);
    memcpy(pointer, src, size);
    VirtualProtect(pointer, size, PAGE_EXECUTE_READ, &old_protection);
    return pointer;
}
