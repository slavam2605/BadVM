#ifndef BADJVM_CODE_BUILDER_H
#define BADJVM_CODE_BUILDER_H

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <cstdint>
#include <ostream>
#include <variant>
#include <optional>

enum class jit_register64 {
    rax, rbx, rcx, rdx, rsp, rbp, rsi, rdi,
    r8, r9, r10, r11, r12, r13, r14, r15,

    xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7,
    xmm8, xmm9, xmm10, xmm11, xmm12, xmm13, xmm14, xmm15,

    no_register
};

std::ostream& operator<<(std::ostream& stream, const jit_register64& reg);

struct jit_value_location {
    jit_register64 reg;
    int32_t stack_offset;
    int bit_size;

    jit_value_location() : reg(jit_register64::no_register), stack_offset(0), bit_size(0) {}

    jit_value_location(jit_register64 reg) {
        this->reg = reg;
        this->stack_offset = 0;
        this->bit_size = 64;
    }

    jit_value_location(jit_register64 reg, int bit_size) {
        this->reg = reg;
        this->stack_offset = 0;
        this->bit_size = bit_size;
    }

    jit_value_location(int32_t stack_offset, int bit_size) {
        this->reg = jit_register64::no_register;
        this->stack_offset = stack_offset;
        this->bit_size = bit_size;
    }

    bool operator==(const jit_value_location& other) const;
    bool operator!=(const jit_value_location& other) const;
};

struct fix_jump_entry {
    int label_offset;
    int jump_source_offset;
    int label_id;

    fix_jump_entry(int labelOffset, int jumpSourceOffset, int labelId)
            : label_offset(labelOffset), jump_source_offset(jumpSourceOffset), label_id(labelId) {}
};

struct fix_double_const_entry {
    int disp_offset;
    int base_offset;
    double value;

    fix_double_const_entry(int disp_offset, int base_offset, double value)
            : disp_offset(disp_offset), base_offset(base_offset), value(value) {}
};

class code_builder {
    std::vector<uint8_t> code;
    std::vector<fix_jump_entry> fix_list;
    std::unordered_map<int, int> label_map; // label id -> native offset
    std::vector<fix_double_const_entry> fix_double_list;
    std::unordered_map<double, int> double_const_offset;

    void push_imm8(uint8_t value);
    void push_imm32(uint32_t value);
    void push_imm64(uint64_t value);
    void write_imm32(int offset, uint32_t value);
    void jcc32(uint8_t opcode, int label_id);
    void internal_push_instruction(std::optional<uint8_t> prefix, bool rex_w, std::optional<jit_value_location> rm,
                                   std::variant<jit_value_location, uint8_t> r, bool r_in_opcode,
                                   std::optional<std::pair<uint64_t, uint8_t>> imm, std::vector<uint8_t> opcodes);
    void instr_rexw_rm_r(jit_value_location rm, jit_value_location r, uint8_t opcode);
    void instr_rexw_rm_r(jit_value_location rm, jit_value_location r, uint8_t opcode1, uint8_t opcode2);
    void instr_prefix_rm_r(uint8_t prefix, jit_value_location rm, jit_value_location r, uint8_t opcode1, uint8_t opcode2);
    void instr_rexw_rm_r_imm32(jit_value_location rm, jit_value_location r, uint8_t opcode, uint32_t imm);
    void instr_rexw_rm_rdigit_imm32(jit_value_location rm, uint8_t reg_digit, uint8_t opcode, uint32_t imm);
    void instr_rexw_rm_rdigit_imm8(jit_value_location rm, uint8_t reg_digit, uint8_t opcode, uint8_t imm);
    void instr_rexw_rm_rdigit(jit_value_location rm, uint8_t reg_digit, uint8_t opcode);
    void instr_rexw_r_in_opcode_imm64(jit_value_location r, uint8_t opcode, uint64_t imm);
    void instr_rexw_r_in_opcode_imm32(jit_value_location r, uint8_t opcode, uint32_t imm);
public:
    const std::vector<uint8_t>& get_code() const;
    int current_offset() const;
    void mark_label(int label_id);
    void resolve_labels();
    void link_constants();

    void mov(jit_value_location from, jit_value_location to);
    void mov(int64_t from, jit_value_location to);
    void mov(int32_t from, jit_value_location to);
    void movsx(jit_value_location from, jit_value_location to);
    void cmovl(jit_value_location from, jit_value_location to);
    void cmovg(jit_value_location from, jit_value_location to);
    void cmovns(jit_value_location from, jit_value_location to);
    void add(jit_value_location from, jit_value_location to);
    void add(int32_t from, jit_value_location to);
    void sub(jit_value_location from, jit_value_location to);
    void sub(int32_t from, jit_value_location to);
    void imul(jit_value_location value);
    void imul(jit_value_location from, jit_value_location to);
    void imul(jit_value_location first, int32_t second, jit_value_location to);
    void idiv(jit_value_location value);
    void neg(jit_value_location value);
    void shr(jit_value_location first, int8_t second);
    void sar(jit_value_location first, int8_t second);
    void cqo();
    void cmp(jit_value_location first, jit_value_location second);
    void cmp(jit_value_location first, int32_t second);
    void test(jit_value_location first, jit_value_location second);
    void movsd(jit_value_location from, jit_value_location to);
    void movsd(double from, jit_value_location to);
    void addsd(jit_value_location from, jit_value_location to);
    void addsd(double from, jit_value_location to);
    void subsd(jit_value_location from, jit_value_location to);
    void mulsd(jit_value_location from, jit_value_location to);
    void mulsd(double from, jit_value_location to);
    void comisd(jit_value_location first, double second);
    void je(int label_id);
    void jne(int label_id);
    void jl(int label_id);
    void jle(int label_id);
    void jg(int label_id);
    void jge(int label_id);
    void ja(int label_id);
    void jb(int label_id);
    void jmp(int label_id);
    void ret();
};


#endif //BADJVM_CODE_BUILDER_H
