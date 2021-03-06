#include "ir_compiler.h"
#include "register_shortcuts.h"

using namespace std;

void ir_compiler::compile_int32_power2_div(jit_value_location first, int32_t second, jit_value_location to, const ir_data_flow_pair& data) {
    auto abs_second = abs(second);
    auto power = __builtin_ctz(abs_second);
    assert(first.reg != no_register)
    jit_value_location temp = first;
    if (first.reg == to.reg) {
        temp = jit_value_location(get_temp_register(data, ir_storage_type::ir_int), first.bit_size);
        compile_assign(first, temp);
    }
    compile_assign(first, to);
    builder.add(abs_second - 1, to);
    builder.test(temp, temp);
    builder.cmovns(temp, to);
    builder.sar(to, power);
    if (second < 0) {
        builder.neg(to);
    }
}

pair<int, int> find_magic(int32_t d) {
    int32_t p;
    uint32_t ad, anc, delta, q1, r1, q2, r2, t;
    constexpr uint32_t two31 = 0x80000000;
    int32_t magic, shift;

    ad = abs(d);
    t = two31 + (static_cast<uint32_t>(d) >> 31);
    anc = t - 1 - t % ad;
    p = 31;
    q1 = two31 / anc;
    r1 = two31 - q1 * anc;
    q2 = two31 / ad;
    r2 = two31 - q2 * ad;
    do {
        p++;
        q1 = 2 * q1;
        r1 = 2 * r1;
        if (r1 >= anc) {
            q1 = q1 + 1;
            r1 = r1 - anc;
        }
        q2 = 2 * q2;
        r2 = 2 * r2;
        if (r2 >= ad) {
            q2 = q2 + 1;
            r2 = r2 - ad;
        }
        delta = ad - r2;
    } while (q1 < delta || (q1 == delta && r1 == 0));
    magic = q2 + 1;
    if (d < 0) magic *= -1;
    shift = p - 32;
    return make_pair(magic, shift);
}

void ir_compiler::compile_int32_div_const(jit_value_location first, int32_t second, jit_value_location to, const ir_data_flow_pair& data) {
    if (__builtin_popcount(abs(second)) == 1) {
        compile_int32_power2_div(first, second, to, data);
        return;
    }

    auto [magic, shift] = find_magic(second);
    bool magic_overflow = magic > 0 != second > 0;

    auto to64 = jit_value_location(to.reg, 64);

    builder.movsx(first, to64);
    builder.imul(to64, magic, to64);
    builder.sar(to64, !magic_overflow ? 32 + shift : 32);
    if (magic_overflow) {
        if (second > 0)
            builder.add(first, to);
        else
            builder.sub(first, to);
    }
    if (shift > 0 && magic_overflow) {
        builder.sar(to, shift);
    }
    if (second > 0) {
        auto temp = jit_value_location(get_temp_register(data, ir_storage_type::ir_int, first.reg), 32);
        if (first.reg != temp.reg) builder.mov(first, temp);
        builder.shr(temp, 31);
        builder.add(temp, to);
    } else {
        auto temp = jit_value_location(get_temp_register(data, ir_storage_type::ir_int), 32);
        builder.mov(to, temp);
        builder.shr(temp, 31);
        builder.add(temp, to);
    }
}

void ir_compiler::compile_int_rem(jit_value_location first, jit_value_location second, jit_value_location to, const ir_data_flow_pair& data) {
    auto bit_size = first.bit_size;
    auto rax_loc = jit_value_location(rax, bit_size);
    auto rdx_loc = jit_value_location(rdx, bit_size);
    bool rax_live = get_temp_register(data, ir_storage_type::ir_int, rax) != rax;
    bool rdx_live = get_temp_register(data, ir_storage_type::ir_int, rdx) != rdx;

    auto temp_second = second;
    jit_register64 temp1 = no_register, temp2 = no_register;
    if (rdx_live && to.reg != rdx) {
        temp1 = get_temp_register(data, ir_storage_type::ir_int, no_register, {rax, rdx, first.reg, second.reg});
        builder.mov(rdx, temp1);
    }
    if (rax_live && to.reg != rax) {
        temp2 = get_temp_register(data, ir_storage_type::ir_int, no_register, {rax, rdx, first.reg, second.reg, temp1});
        builder.mov(rax, temp2);
    }
    if (second.reg == rdx && temp1 != no_register) {
        temp_second = jit_value_location(temp1, bit_size);
    } else if (second.reg == rax && temp2 != no_register) {
        temp_second = jit_value_location(temp2, bit_size);
    } else if (second.reg == to.reg) {
        auto temp_reg = get_temp_register(data, ir_storage_type::ir_int, no_register, {rax, rdx, first.reg, second.reg, temp1, temp2});
        temp_second = jit_value_location(temp_reg, bit_size);
        compile_assign(second, temp_second);
    }
    compile_assign(first, rax_loc);
    builder.cqo();
    builder.idiv(temp_second);
    compile_assign(rdx_loc, to);
    if (temp1 != no_register) {
        builder.mov(temp1, rdx);
    }
    if (temp2 != no_register) {
        builder.mov(temp2, rax);
    }
}