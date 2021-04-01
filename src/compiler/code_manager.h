#ifndef BADJVM_CODE_MANAGER_H
#define BADJVM_CODE_MANAGER_H

#include <vector>
#include <cstdint>

class code_manager {
    const uint64_t code_cache_size = 100000;
    uint8_t* code_cache;
    uint64_t offset = 0;

public:
    code_manager();
    ~code_manager();
    uint8_t* add_code_chunk(const std::vector<uint8_t>& code);
    uint8_t* add_code_chunk(const void* src, uint64_t size);
};


#endif //BADJVM_CODE_MANAGER_H
