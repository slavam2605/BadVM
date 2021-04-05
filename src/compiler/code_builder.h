#ifndef BADJVM_CODE_BUILDER_H
#define BADJVM_CODE_BUILDER_H

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <cstdint>
#include <ostream>

enum class jit_register64 {
    rax, rbx, rcx, rdx, rsp, rbp, rsi, rdi,
    r8, r9, r10, r11, r12, r13, r14, r15,
    no_register
};

std::ostream& operator<<(std::ostream& stream, const jit_register64& reg);

struct jit_value_location {
    jit_register64 reg;
    int32_t stack_offset;
    
    jit_value_location(jit_register64 reg) {
        this->reg = reg;
        this->stack_offset = 0;
    }

    jit_value_location(int32_t stack_offset) {
        this->reg = jit_register64::no_register;
        this->stack_offset = stack_offset;
    }

    bool operator==(const jit_value_location& other) const;
};

struct fix_jump_entry {
    int label_offset;
    int jump_source_offset;
    int label_id;

    fix_jump_entry(int labelOffset, int jumpSourceOffset, int labelId)
            : label_offset(labelOffset), jump_source_offset(jumpSourceOffset), label_id(labelId) {}
};

class code_builder {
    std::vector<uint8_t> code;
    std::vector<fix_jump_entry> fix_list;
    std::unordered_map<int, int> label_map; // label id -> native offset

    void push_imm32(uint32_t value);
    void push_imm64(uint64_t value);
    void write_imm32(int offset, uint32_t value);
    void jcc32(uint8_t opcode, int label_id);
public:
    const std::vector<uint8_t>& get_code() const;
    int current_offset() const;
    void mark_label(int label_id);
    void resolve_labels();

    void mov(jit_value_location from, jit_value_location to);
    void mov(int64_t from, jit_value_location to);
    void add(jit_value_location from, jit_value_location to);
    void add(int32_t from, jit_value_location to);
    void sub(jit_value_location from, jit_value_location to);
    void sub(int32_t from, jit_value_location to);
    void imul(jit_value_location from, jit_value_location to);
    void idiv(jit_value_location value);
    void neg(jit_value_location value);
    void cqo();
    void cmp(jit_value_location first, jit_value_location second);
    void cmp(jit_value_location first, int32_t second);
    void je(int label_id);
    void jne(int label_id);
    void jl(int label_id);
    void jle(int label_id);
    void jg(int label_id);
    void jge(int label_id);
    void jmp(int label_id);
    void ret();
};


#endif //BADJVM_CODE_BUILDER_H
