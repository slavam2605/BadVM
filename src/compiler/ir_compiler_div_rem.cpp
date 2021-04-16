#include "ir_compiler.h"
#include "register_shortcuts.h"

using namespace std;

void ir_compiler::compile_int32_power2_div(jit_value_location first, int32_t second, jit_value_location to) {
    auto abs_second = abs(second);
    auto power = __builtin_ctz(abs_second);
    // TODO allocate temp register
    assert(first.reg != no_register)
    jit_value_location temp = first;
    if (first.reg == to.reg) {
        compile_assign(first, jit_value_location(r11, first.bit_size));
        temp = jit_value_location(r11, first.bit_size);
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

void ir_compiler::compile_int32_div(jit_value_location first, int32_t second, jit_value_location to) {
    if (__builtin_popcount(abs(second)) == 1) {
        compile_int32_power2_div(first, second, to);
        return;
    }

    auto [magic, shift] = find_magic(second);
    bool magic_overflow = magic > 0 != second > 0;

    auto to64 = jit_value_location(to.reg, 64);
    auto r11d = jit_value_location(r11, 32);

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
    // TODO allocate temp register or check liveness of first after this instruction
    if (second > 0) {
        builder.mov(first, r11d);
    } else {
        builder.mov(to, r11d);
    }
    builder.shr(r11d, 31);
    builder.add(r11d, to);
}