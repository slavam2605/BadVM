#include "code_builder.h"
#include "../utils/utils.h"
#include <iostream>

constexpr uint8_t REX_W = 0b01001000;
constexpr uint8_t REX_R = 0b01000100;
constexpr uint8_t REX_X = 0b01000010;
constexpr uint8_t REX_B = 0b01000001;

using namespace std;

#define DEBUG
#ifdef DEBUG
#define log(x) x
#else
#define log(x)
#endif

ostream& operator<<(ostream& stream, const jit_value_location& loc) {
    switch (loc.reg) {
        case jit_register64::rax: stream << "rax"; break;
        case jit_register64::rbx: stream << "rbx"; break;
        case jit_register64::rcx: stream << "rcx"; break;
        case jit_register64::rdx: stream << "rdx"; break;
        case jit_register64::rsp: stream << "rsp"; break;
        case jit_register64::rbp: stream << "rbp"; break;
        case jit_register64::rsi: stream << "rsi"; break;
        case jit_register64::rdi: stream << "rdi"; break;
        case jit_register64::r8:  stream << "r8";  break;
        case jit_register64::r9:  stream << "r9";  break;
        case jit_register64::r10: stream << "r10"; break;
        case jit_register64::r11: stream << "r11"; break;
        case jit_register64::r12: stream << "r12"; break;
        case jit_register64::r13: stream << "r13"; break;
        case jit_register64::r14: stream << "r14"; break;
        case jit_register64::r15: stream << "r15"; break;
        case jit_register64::no_register: stream << "[stack offset = " << loc.stack_offset << "]"; break;
    }
    return stream;
}

bool jit_value_location::operator==(const jit_value_location& other) const {
    return reg == other.reg && stack_offset == other.stack_offset;
}

const vector<uint8_t>& code_builder::get_code() const {
    return code;
}

int code_builder::current_offset() const {
    return code.size();
}

void code_builder::mark_label(int label_id) {
    log(cout << "L" << label_id << ":" << endl;)
    label_map[label_id] = current_offset();
}

uint8_t get_register_index_long(uint8_t& rex, uint8_t rex_flag, jit_register64 reg) {
    switch (reg) {
        case jit_register64::rax: return 0b000;
        case jit_register64::rcx: return 0b001;
        case jit_register64::rdx: return 0b010;
        case jit_register64::rbx: return 0b011;
        case jit_register64::rsp: return 0b100;
        case jit_register64::rbp: return 0b101;
        case jit_register64::rsi: return 0b110;
        case jit_register64::rdi: return 0b111;
        case jit_register64::r8:  rex |= rex_flag; return 0b000;
        case jit_register64::r9:  rex |= rex_flag; return 0b001;
        case jit_register64::r10: rex |= rex_flag; return 0b010;
        case jit_register64::r11: rex |= rex_flag; return 0b011;
        case jit_register64::r12: rex |= rex_flag; return 0b100;
        case jit_register64::r13: rex |= rex_flag; return 0b101;
        case jit_register64::r14: rex |= rex_flag; return 0b110;
        case jit_register64::r15: rex |= rex_flag; return 0b111;
        default: assert(false)
    }
}

uint8_t get_register_index(jit_register64 reg) {
    uint8_t rex = 0;
    auto result = get_register_index_long(rex, 1, reg);
    assert(rex == 0)
    return result;
}

uint8_t create_mod_rm(uint8_t& rex, jit_value_location rm, jit_value_location r) {
    auto first_reg = rm.reg;
    auto second_reg = r.reg;
    assert(first_reg != jit_register64::no_register)
    assert(second_reg != jit_register64::no_register)

    auto rm_index = get_register_index_long(rex, REX_B, first_reg);
    auto reg_index = get_register_index_long(rex, REX_R, second_reg);

    uint8_t reg, mod_rm;
    mod_rm = 0b11000000 | rm_index;
    reg = reg_index << 3;
    return reg | mod_rm;
}

uint8_t create_mod_rm_reg_digit(jit_value_location rm, uint8_t digit) {
    auto rm_reg = rm.reg;
    assert(rm_reg != jit_register64::no_register)

    uint8_t dummy;
    auto rm_index = get_register_index_long(dummy, 0, rm_reg);

    return (digit << 3) | 0b11000000 | rm_index;
}

void code_builder::push_imm64(uint64_t value) {
    code.push_back(value & 0xFF);
    code.push_back((value >> 8) & 0xFF);
    code.push_back((value >> 16) & 0xFF);
    code.push_back((value >> 24) & 0xFF);
    code.push_back((value >> 32) & 0xFF);
    code.push_back((value >> 40) & 0xFF);
    code.push_back((value >> 48) & 0xFF);
    code.push_back((value >> 56) & 0xFF);
}

void code_builder::push_imm32(uint32_t value) {
    code.push_back(value & 0xFF);
    code.push_back((value >> 8) & 0xFF);
    code.push_back((value >> 16) & 0xFF);
    code.push_back((value >> 24) & 0xFF);
}

void code_builder::write_imm32(int offset, uint32_t value) {
    code[offset] = value & 0xFF;
    code[offset + 1] = (value >> 8) & 0xFF;
    code[offset + 2] = (value >> 16) & 0xFF;
    code[offset + 3] = (value >> 24) & 0xFF;
}

void code_builder::mov(jit_value_location from, jit_value_location to) {
    // TODO maybe assert from != to?
    log(cout << "    mov " << to << ", " << from << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, to, from);

    code.push_back(rex);
    code.push_back(0x89);
    code.push_back(mod_rm);
}

void code_builder::mov(int64_t from, jit_value_location to) {
    log(cout << "    mov " << to << ", " << from << endl;)
    assert(to.reg != jit_register64::no_register)
    auto rex = REX_W;
    auto reg_index = get_register_index_long(rex, REX_B, to.reg);

    code.push_back(rex);
    code.push_back(0xB8 | reg_index);
    push_imm64(from);
}

void code_builder::add(jit_value_location from, jit_value_location to) {
    log(cout << "    add " << to << ", " << from << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, to, from);

    code.push_back(rex);
    code.push_back(0x01);
    code.push_back(mod_rm);
}

void code_builder::add(int32_t from, jit_value_location to) {
    // from is sign-extended to `to` register
    log(cout << "    add " << to << ", " << from << endl;)
    auto mod_rm = create_mod_rm_reg_digit(to, 0);

    code.push_back(REX_W);
    code.push_back(0x81);
    code.push_back(mod_rm);
    push_imm32(from);
}

void code_builder::sub(jit_value_location from, jit_value_location to) {
    log(cout << "    sub " << to << ", " << from << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, to, from);

    code.push_back(rex);
    code.push_back(0x29);
    code.push_back(mod_rm);
}

void code_builder::cmp(jit_value_location first, jit_value_location second) {
    log(cout << "    cmp " << first << ", " << second << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, first, second);

    code.push_back(rex);
    code.push_back(0x39);
    code.push_back(mod_rm);
}

void code_builder::jge(int label_id) {
    log(cout << "    jge L" << label_id << endl;)
    code.push_back(0x0F);
    code.push_back(0x8D);
    push_imm32(0);
    fix_list.emplace_back(current_offset() - 4, current_offset(), label_id);
}

void code_builder::jmp(int label_id) {
    log(cout << "    jmp L" << label_id << endl;)
    code.push_back(0xE9);
    push_imm32(0);
    fix_list.emplace_back(current_offset() - 4, current_offset(), label_id);
}

void code_builder::ret() {
    log(cout << "    ret" << endl;)
    code.push_back(0xC3);
}

void code_builder::resolve_labels() {
    for (const auto& [label_offset, jump_src_offset, label_id] : fix_list) {
        auto iter = label_map.find(label_id);
        assert(iter != label_map.end());
        int32_t jump_rel_offset = iter->second - jump_src_offset;
        write_imm32(label_offset, jump_rel_offset);
    }
}