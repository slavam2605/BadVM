#include "code_builder.h"
#include "../utils/utils.h"
#include <iostream>

constexpr uint8_t REX   = 0b01000000;
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
    if (loc.reg == jit_register64::no_register) {
        return stream << "[stack offset = " << loc.stack_offset << ", size = " << loc.bit_size << " bits]";
    }

    switch (loc.bit_size) {
        case 64: {
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
                case jit_register64::xmm0: stream << "xmm0"; break;
                case jit_register64::xmm1: stream << "xmm1"; break;
                case jit_register64::xmm2: stream << "xmm2"; break;
                case jit_register64::xmm3: stream << "xmm3"; break;
                case jit_register64::xmm4: stream << "xmm4"; break;
                case jit_register64::xmm5: stream << "xmm5"; break;
                case jit_register64::xmm6: stream << "xmm6"; break;
                case jit_register64::xmm7: stream << "xmm7"; break;
                case jit_register64::xmm8: stream << "xmm8"; break;
                case jit_register64::xmm9: stream << "xmm9"; break;
                case jit_register64::xmm10: stream << "xmm10"; break;
                case jit_register64::xmm11: stream << "xmm11"; break;
                case jit_register64::xmm12: stream << "xmm12"; break;
                case jit_register64::xmm13: stream << "xmm13"; break;
                case jit_register64::xmm14: stream << "xmm14"; break;
                case jit_register64::xmm15: stream << "xmm15"; break;
                default_fail
            }
            break;
        }
        case 32: {
            switch (loc.reg) {
                case jit_register64::rax: stream << "eax"; break;
                case jit_register64::rbx: stream << "ebx"; break;
                case jit_register64::rcx: stream << "ecx"; break;
                case jit_register64::rdx: stream << "edx"; break;
                case jit_register64::rsp: stream << "esp"; break;
                case jit_register64::rbp: stream << "ebp"; break;
                case jit_register64::rsi: stream << "esi"; break;
                case jit_register64::rdi: stream << "edi"; break;
                case jit_register64::r8:  stream << "r8d";  break;
                case jit_register64::r9:  stream << "r9d";  break;
                case jit_register64::r10: stream << "r10d"; break;
                case jit_register64::r11: stream << "r11d"; break;
                case jit_register64::r12: stream << "r12d"; break;
                case jit_register64::r13: stream << "r13d"; break;
                case jit_register64::r14: stream << "r14d"; break;
                case jit_register64::r15: stream << "r15d"; break;
                default_fail
            }
            break;
        }
        default_fail
    }
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const jit_register64& reg) {
    return stream << jit_value_location(reg);
}

bool jit_value_location::operator==(const jit_value_location& other) const {
    return reg == other.reg && stack_offset == other.stack_offset && bit_size == other.bit_size;
}

bool jit_value_location::operator!=(const jit_value_location& other) const {
    return !(*this == other);
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
        case jit_register64::xmm0:  return 0b000;
        case jit_register64::xmm1:  return 0b001;
        case jit_register64::xmm2:  return 0b010;
        case jit_register64::xmm3:  return 0b011;
        case jit_register64::xmm4:  return 0b100;
        case jit_register64::xmm5:  return 0b101;
        case jit_register64::xmm6:  return 0b110;
        case jit_register64::xmm7:  return 0b111;
        case jit_register64::xmm8:  rex |= rex_flag; return 0b000;
        case jit_register64::xmm9:  rex |= rex_flag; return 0b001;
        case jit_register64::xmm10: rex |= rex_flag; return 0b010;
        case jit_register64::xmm11: rex |= rex_flag; return 0b011;
        case jit_register64::xmm12: rex |= rex_flag; return 0b100;
        case jit_register64::xmm13: rex |= rex_flag; return 0b101;
        case jit_register64::xmm14: rex |= rex_flag; return 0b110;
        case jit_register64::xmm15: rex |= rex_flag; return 0b111;
        default_fail
    }
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

uint8_t create_mod_rm_reg_digit(uint8_t& rex, jit_value_location rm, uint8_t digit) {
    auto rm_reg = rm.reg;
    assert(rm_reg != jit_register64::no_register)

    auto rm_index = get_register_index_long(rex, REX_B, rm_reg);

    return (digit << 3) | 0b11000000 | rm_index;
}

void code_builder::push_mod_rm_rip_relative_displacement(jit_value_location to, int32_t displacement) {
    assert(to.reg != jit_register64::no_register)
    uint8_t rex = 0;
    uint8_t reg = get_register_index_long(rex, 1, to.reg);
    assert(rex == 0)
    code.push_back(0b00000101 | (reg << 3));
    push_imm32(displacement);
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
    auto rex = REX_W;
    auto reg_index = get_register_index_long(rex, REX_B, to.reg);

    code.push_back(rex);
    code.push_back(0xB8 | reg_index);
    push_imm64(from);
}

void movsx32_64(vector<uint8_t>& code, jit_value_location from, jit_value_location to) {
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, from, to);

    code.push_back(rex);
    code.push_back(0x63);
    code.push_back(mod_rm);
}

void code_builder::movsx(jit_value_location from, jit_value_location to) {
    log(cout << "    movsx " << to << ", " << from << endl;)
    switch (to.bit_size) {
        case 64: {
            switch (from.bit_size) {
                case 32: movsx32_64(code, from, to); break;
                default_fail
            }
            break;
        }
        default_fail
    }
}

void code_builder::cmovl(jit_value_location from, jit_value_location to) {
    log(cout << "    cmovl " << to << ", " << from << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, from, to);

    code.push_back(rex);
    code.push_back(0x0F);
    code.push_back(0x4C);
    code.push_back(mod_rm);
}

void code_builder::cmovg(jit_value_location from, jit_value_location to) {
    log(cout << "    cmovg " << to << ", " << from << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, from, to);

    code.push_back(rex);
    code.push_back(0x0F);
    code.push_back(0x4F);
    code.push_back(mod_rm);
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
    // from is sign-extended to 64 bits
    log(cout << "    add " << to << ", " << from << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm_reg_digit(rex, to, 0);

    code.push_back(rex);
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

void code_builder::sub(int32_t from, jit_value_location to) {
    // from is sign-extended to 64 bits
    log(cout << "    sub " << to << ", " << from << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm_reg_digit(rex, to, 5);

    code.push_back(rex);
    code.push_back(0x81);
    code.push_back(mod_rm);
    push_imm32(from);
}

void code_builder::imul(jit_value_location from, jit_value_location to) {
    log(cout << "    imul " << to << ", " << from << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, from, to);

    code.push_back(rex);
    code.push_back(0x0F);
    code.push_back(0xAF);
    code.push_back(mod_rm);
}

void code_builder::imul(jit_value_location first, int32_t second, jit_value_location to) {
    log(cout << "    imul " << to << ", " << first << ", " << second << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, first, to);

    code.push_back(rex);
    code.push_back(0x69);
    code.push_back(mod_rm);
    push_imm32(second);
}

void code_builder::idiv(jit_value_location value) {
    log(cout << "    idiv " << value << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm_reg_digit(rex, value, 7);

    code.push_back(rex);
    code.push_back(0xF7);
    code.push_back(mod_rm);
}

void code_builder::neg(jit_value_location value) {
    log(cout << "    neg " << value << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm_reg_digit(rex, value, 3);

    code.push_back(rex);
    code.push_back(0xF7);
    code.push_back(mod_rm);
}

void code_builder::cqo() {
    log(cout << "    cqo" << endl;)
    code.push_back(REX_W);
    code.push_back(0x99);
}

void code_builder::cmp(jit_value_location first, jit_value_location second) {
    log(cout << "    cmp " << first << ", " << second << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm(rex, first, second);

    code.push_back(rex);
    code.push_back(0x39);
    code.push_back(mod_rm);
}

void code_builder::cmp(jit_value_location first, int32_t second) {
    // second it sign-extended to 64 bits
    log(cout << "    cmp " << first << ", " << second << endl;)
    auto rex = REX_W;
    auto mod_rm = create_mod_rm_reg_digit(rex, first, 7);

    code.push_back(rex);
    code.push_back(0x81);
    code.push_back(mod_rm);
    push_imm32(second);
}

void code_builder::jcc32(uint8_t opcode, int label_id) {
    code.push_back(0x0F);
    code.push_back(opcode);
    push_imm32(0);
    fix_list.emplace_back(current_offset() - 4, current_offset(), label_id);
}

void code_builder::movsd(jit_value_location from, jit_value_location to) {
    log(cout << "    movsd " << to << ", " << from << endl;)
    auto rex = REX;
    auto mod_rm = create_mod_rm(rex, from, to);

    code.push_back(0xF2);
    if (rex != REX) code.push_back(rex);
    code.push_back(0x0F);
    code.push_back(0x10);
    code.push_back(mod_rm);
}

void code_builder::movsd(double from, jit_value_location to) {
    log(cout << "    movsd " << to << ", " << from << endl;)
    if (double_const_offset.find(from) == double_const_offset.end()) {
        double_const_offset[from] = 0;
    }

    code.push_back(0xF2);
    code.push_back(0x0F);
    code.push_back(0x10);
    push_mod_rm_rip_relative_displacement(to, 0);

    fix_double_list.emplace_back(current_offset() - 4, current_offset(), from);
}

void code_builder::mulsd(jit_value_location from, jit_value_location to) {
    log(cout << "    mulsd " << to << ", " << from << endl;)
    auto rex = REX;
    auto mod_rm = create_mod_rm(rex, from, to);

    code.push_back(0xF2);
    if (rex != REX) code.push_back(rex);
    code.push_back(0x0F);
    code.push_back(0x59);
    code.push_back(mod_rm);
}

void code_builder::je(int label_id) {
    log(cout << "    je L" << label_id << endl;)
    jcc32(0x84, label_id);
}

void code_builder::jne(int label_id) {
    log(cout << "    jne L" << label_id << endl;)
    jcc32(0x85, label_id);
}

void code_builder::jl(int label_id) {
    log(cout << "    jl L" << label_id << endl;)
    jcc32(0x8C, label_id);
}

void code_builder::jle(int label_id) {
    log(cout << "    jle L" << label_id << endl;)
    jcc32(0x8E, label_id);
}

void code_builder::jg(int label_id) {
    log(cout << "    jg L" << label_id << endl;)
    jcc32(0x8F, label_id);
}

void code_builder::jge(int label_id) {
    log(cout << "    jge L" << label_id << endl;)
    jcc32(0x8D, label_id);
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

void code_builder::link_constants() {
    for (auto& [value, offset] : double_const_offset) {
        offset = current_offset();
        uint64_t uint64_blob = *reinterpret_cast<const uint64_t*>(&value);
        push_imm64(uint64_blob);
        log(cout << ".quad " << (int64_t) uint64_blob << "    # double " << value << endl;)
    }

    for (const auto& [disp_offset, base_offset, value] : fix_double_list) {
        auto iter = double_const_offset.find(value);
        assert(iter != double_const_offset.end());
        auto target_offset = iter->second;
        write_imm32(disp_offset, target_offset - base_offset);
    }
}