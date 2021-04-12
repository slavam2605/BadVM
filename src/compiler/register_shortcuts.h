#ifndef BADJVM_REGISTER_SHORTCUTS_H
#define BADJVM_REGISTER_SHORTCUTS_H

#include "code_builder.h"

auto static constexpr rax = jit_register64::rax;
auto static constexpr rbx = jit_register64::rbx;
auto static constexpr rcx = jit_register64::rcx;
auto static constexpr rdx = jit_register64::rdx;
auto static constexpr rsp = jit_register64::rsp;
auto static constexpr rbp = jit_register64::rbp;
auto static constexpr rsi = jit_register64::rsi;
auto static constexpr rdi = jit_register64::rdi;
auto static constexpr r8  = jit_register64::r8;
auto static constexpr r9  = jit_register64::r9;
auto static constexpr r10 = jit_register64::r10;
auto static constexpr r11 = jit_register64::r11;
auto static constexpr r12 = jit_register64::r12;
auto static constexpr r13 = jit_register64::r13;
auto static constexpr r14 = jit_register64::r14;
auto static constexpr r15 = jit_register64::r15;

auto static constexpr xmm0 = jit_register64::xmm0;
auto static constexpr xmm1 = jit_register64::xmm1;
auto static constexpr xmm2 = jit_register64::xmm2;
auto static constexpr xmm3 = jit_register64::xmm3;
auto static constexpr xmm4 = jit_register64::xmm4;
auto static constexpr xmm5 = jit_register64::xmm5;
auto static constexpr xmm6 = jit_register64::xmm6;
auto static constexpr xmm7 = jit_register64::xmm7;
auto static constexpr xmm8 = jit_register64::xmm8;
auto static constexpr xmm9 = jit_register64::xmm9;
auto static constexpr xmm10 = jit_register64::xmm10;
auto static constexpr xmm11 = jit_register64::xmm11;
auto static constexpr xmm12 = jit_register64::xmm12;
auto static constexpr xmm13 = jit_register64::xmm13;
auto static constexpr xmm14 = jit_register64::xmm14;
auto static constexpr xmm15 = jit_register64::xmm15;

auto static constexpr no_register = jit_register64::no_register;

#endif //BADJVM_REGISTER_SHORTCUTS_H
