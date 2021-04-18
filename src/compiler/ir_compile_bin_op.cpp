#include "ir_compiler.h"
#include "register_shortcuts.h"
#include "../utils/utils.h"

using namespace std;

void ir_compiler::compile_double_bin_op(const std::shared_ptr<ir_bin_op_insruction>& instruction, const ir_data_flow_pair& data) {
    auto op = instruction->op;
    auto first_value = instruction->first;
    auto second_value = instruction->second;
    auto to = get_location(instruction->to);
    if (first_value.mode != ir_value_mode::var && second_value.mode == ir_value_mode::var &&
        (op == ir_bin_op::add || op == ir_bin_op::mul)) {
        swap(first_value, second_value);
    }

    switch (first_value.mode) {
        case ir_value_mode::var: {
            auto first = get_location(first_value.var);
            switch (second_value.mode) {
                case ir_value_mode::var: {
                    auto second = get_location(second_value.var);
                    switch (op) {
                        case ir_bin_op::add: {
                            if (first == to) {
                                builder.addsd(second, to);
                            } else if (second == to) {
                                builder.addsd(first, to);
                            } else {
                                builder.movsd(first, to);
                                builder.addsd(second, to);
                            };
                            break;
                        }
                        case ir_bin_op::sub: {
                            if (first == to) {
                                builder.subsd(second, to);
                            } else if (second == to) {
                                auto temp = get_temp_register(data, ir_storage_type::ir_float, first.reg);
                                if (temp == first.reg) {
                                    builder.subsd(to, temp);
                                    builder.movsd(temp, to);
                                } else {
                                    builder.movsd(to, temp);
                                    builder.movsd(first, to);
                                    builder.subsd(temp, to);
                                }
                            } else {
                                builder.movsd(first, to);
                                builder.subsd(second, to);
                            };
                            break;
                        }
                        case ir_bin_op::mul: {
                            if (first == to) {
                                builder.mulsd(second, to);
                            } else if (second == to) {
                                builder.mulsd(first, to);
                            } else {
                                builder.movsd(first, to);
                                builder.mulsd(second, to);
                            };
                            break;
                        }
                        case ir_bin_op::div: assert(false)
                        case ir_bin_op::rem: assert(false)
                        case ir_bin_op::cmp: assert(false)
                        default_fail
                    }
                    break;
                }
                case ir_value_mode::float64: {
                    auto second = second_value.float64_value;
                    switch (op) {
                        case ir_bin_op::add: {
                            if (first != to) {
                                builder.movsd(first, to);
                            }
                            builder.addsd(second, to);
                            break;
                        }
                        case ir_bin_op::mul: {
                            if (first != to) {
                                builder.movsd(first, to);
                            }
                            if (second == 2.0) {
                                builder.addsd(first, to);
                                break;
                            }
                            builder.mulsd(second, to);
                            break;
                        }
                        case ir_bin_op::cmp: {
                            auto gt_label = create_label();
                            auto lt_label = create_label();
                            auto after_label = create_label();
                            builder.comisd(first, second);
                            builder.ja(gt_label.id);
                            builder.jb(lt_label.id);
                            builder.mov(0, to);
                            builder.jmp(after_label.id);
                            builder.mark_label(gt_label.id);
                            builder.mov(1, to);
                            builder.jmp(after_label.id);
                            builder.mark_label(lt_label.id);
                            builder.mov(-1, to);
                            builder.mark_label(after_label.id);
                            break;
                        }
                        default_fail
                    }
                    break;
                }
                default_fail
            }
            break;
        }
        case ir_value_mode::float64: assert(false)
        default_fail
    }
}

void ir_compiler::compile_bin_op(const shared_ptr<ir_bin_op_insruction>& instruction, const ir_data_flow_pair& data) {
    if (instruction->first.mode == ir_value_mode::float64 ||
        (instruction->first.mode == ir_value_mode::var &&
         get_storage_type(instruction->first.var.type) == ir_storage_type::ir_float)) {
        compile_double_bin_op(instruction, data);
        return;
    }

    auto op = instruction->op;
    auto first_value = instruction->first;
    auto second_value = instruction->second;
    auto to = get_location(instruction->to);
    switch (op) {
        case ir_bin_op::add:
        case ir_bin_op::mul: {
            if ((first_value.mode != ir_value_mode::var && second_value.mode == ir_value_mode::var) ||
                (first_value.mode == ir_value_mode::var && second_value.mode == ir_value_mode::var &&
                        get_location(second_value.var) == to)) {
                swap(first_value, second_value);
            }
            break;
        }
        case ir_bin_op::sub:
        case ir_bin_op::div:
        case ir_bin_op::rem:
        case ir_bin_op::cmp:
            break;
            default_fail
    }

    switch (first_value.mode) {
        case ir_value_mode::var: {
            auto first = get_location(first_value.var);
            switch (second_value.mode) {
                case ir_value_mode::var: {
                    auto second = get_location(second_value.var);
                    switch (op) {
                        case ir_bin_op::add: {
                            if (first == to) {
                                builder.add(second, to);
                            } else {
                                assert(second != to)
                                builder.mov(first, to);
                                builder.add(second, to);
                            }
                            break;
                        }
                        case ir_bin_op::sub: {
                            if (first == to) {
                                builder.sub(second, to);
                            } else if (second == to) {
                                // This is faster then `sub to, first; neg to`
                                builder.neg(to);
                                builder.add(first, to);
                            } else {
                                builder.mov(first, to);
                                builder.sub(second, to);
                            }
                            break;
                        }
                        case ir_bin_op::mul: {
                            if (first != to) {
                                builder.mov(first, to);
                            }
                            builder.imul(second, to);
                            break;
                        }
                        case ir_bin_op::rem: {
                            compile_int_rem(first, second, to, data);
                            break;
                        }
                        case ir_bin_op::cmp: {
                            auto temp = get_temp_register(data, ir_storage_type::ir_int);
                            builder.cmp(first, second);
                            builder.mov(0, to);
                            builder.mov(1, temp);
                            builder.cmovg(temp, to);
                            builder.mov(-1, temp);
                            builder.cmovl(temp, to);
                            break;
                        }
                        default_fail
                    }
                    break;
                }
                case ir_value_mode::int64: {
                    auto value = second_value.int64_value;

                    // TODO implement ir_value_mode::int32
                    if (op == ir_bin_op::div && first_value.var.type == ir_variable_type::ir_int) {
                        compile_int32_div_const(first, static_cast<int32_t>(value), to, data);
                        break;
                    }

                    if (op == ir_bin_op::rem) {
                        // TODO replace idiv with a bit hack and imul
                        // TODO allocate temp register
                        jit_register64 temp = r11;
                        if (to.reg != rdx) {
                            builder.mov(rdx, r11);
                            temp = r10;
                        }
                        if (to.reg != rax) {
                            builder.mov(rax, r10);
                            assert(temp != r10) // TODO can't allocate one more temp register
                        }
                        // TODO movsx for int32
                        compile_assign(first, rax);
                        builder.cqo();
                        builder.mov(value, temp);
                        builder.idiv(temp);
                        compile_assign(rdx, to);
                        if (to.reg != rax) {
                            builder.mov(r10, rax);
                        }
                        if (to.reg != rdx) {
                            builder.mov(r11, rdx);
                        }
                        break;
                    }

                    if (static_cast<int32_t>(value) == value) {
                        if (op == ir_bin_op::mul) {
                            builder.imul(first, value, to);
                            break;
                        }

                        if (first != to) {
                            builder.mov(first, to);
                        }
                        switch (op) {
                            case ir_bin_op::add: builder.add(value, to); break;
                            case ir_bin_op::sub: builder.sub(value, to); break;
                            case ir_bin_op::div:
                            case ir_bin_op::rem:
                            case ir_bin_op::mul: assert(false) // handled above
                            default_fail
                        }
                    } else {
                        if (op == ir_bin_op::mul) {
                            if (first != to) {
                                builder.mov(value, to);
                                builder.imul(first, to);
                            } else {
                                auto temp = get_temp_register(data, ir_storage_type::ir_int);
                                builder.mov(value, temp);
                                builder.imul(temp, to);
                            }
                            break;
                        }

                        if (first != to) {
                            builder.mov(first, to);
                        }
                        switch (op) {
                            case ir_bin_op::add: assert(false)
                            case ir_bin_op::sub: assert(false)
                            case ir_bin_op::rem: assert(false)
                            case ir_bin_op::mul: assert(false) // handled above
                            default_fail
                        }
                    }
                    break;
                }
                default: assert(false)
            }
            break;
        }
        case ir_value_mode::int64: {
            switch (op) {
                case ir_bin_op::sub: {
                    assert(false) // TODO
                    break;
                }
                    default_fail
            }
        }
        default: assert(false)
    }
}