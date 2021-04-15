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

void ir_compiler::compile_int32_div(jit_value_location first, int32_t second, jit_value_location to) {
    if (__builtin_popcount(abs(second)) == 1) {
        compile_int32_power2_div(first, second, to);
        return;
    }
    assert(false)
}