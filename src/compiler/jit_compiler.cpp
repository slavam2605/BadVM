#include "jit_compiler.h"
#include "../utils/constant_pool_utils.h"
#include "../interpreter/method_descriptor.h"
#include "../utils/utils.h"
#include "../interpreter/opcodes.h"
#include "register_shortcuts.h"
#include "jit_type_utils.h"
#include "ir_compiler.h"
#include <iostream>

using namespace std;

#define read_byte1(name) uint8_t name = code[++offset];
#define read_byte2(name) uint8_t name##byte1 = code[++offset]; uint8_t name##byte2 = code[++offset]; uint16_t name = (name##byte1 << 8) | name##byte2;

ir_variable allocate_stack(int locals_count, int& stack_size) {
    int stack_index = stack_size++;
    return ir_variable(locals_count + stack_index);
}

ir_variable pop_stack(int locals_count, int& stack_size) {
    int stack_index = --stack_size;
    return ir_variable(locals_count + stack_index);
}

ir_variable get_local(int index) {
    return ir_variable(index);
}

const void* jit_compiler::compile(const class_file* current_class, const method_info& method, const code_attribute_info* code_info) {
    auto descriptor_string = read_utf8_string(current_class, method.descriptor_index);
    auto descriptor = method_descriptor::parse(descriptor_string);

    int locals_count = code_info->max_locals;
    int stack_size = 0;

//    vector<jit_register64> arguments_regs {rcx, rdx, r8, r9};
//    auto local_offset = 0;
//    auto arg_reg_it = arguments_regs.begin();
//    for (const auto& parameter: descriptor.parameters) {
//        const auto& base_type = parameter.get_base_type();
//        int param_size = 1;
//        if (base_type == base_type_descriptor::long_d || base_type == base_type_descriptor::double_d) {
//            param_size = 2;
//        }
//
//        if (arg_reg_it != arguments_regs.end()) {
//            // TODO support float arguments
//            auto next_reg = *arg_reg_it;
//            locals[local_offset] = next_reg;
//            used_regs.insert(next_reg);
//            free_volatile_regs.erase(next_reg);
//            ++arg_reg_it;
//        } else {
//            assert(false)
//        }
//        local_offset += param_size;
//    }

    ir_compiler ir(manager);

    unordered_map<int, int> bytecode_offset_to_ir;
    unordered_map<int, ir_label> pending_labels;
    const uint8_t* code = code_info->code;
    uint32_t offset = 0;
    while (offset < code_info->code_length) {
        bytecode_offset_to_ir[offset] = ir.ir_offset();
        auto label_iter = pending_labels.find(offset);
        if (label_iter != pending_labels.end()) {
            ir.add_label(label_iter->second, ir.ir_offset());
            pending_labels.erase(offset);
        }

        switch (code[offset]) {
            case op_nop: break;
            case op_iconst_m1: ir.assign(ir_value(-1), allocate_stack(locals_count, stack_size)); break;
            case op_iconst_0: ir.assign(ir_value(0), allocate_stack(locals_count, stack_size)); break;
            case op_iconst_1: ir.assign(ir_value(1), allocate_stack(locals_count, stack_size)); break;
            case op_iconst_2: ir.assign(ir_value(2), allocate_stack(locals_count, stack_size)); break;
            case op_iconst_3: ir.assign(ir_value(3), allocate_stack(locals_count, stack_size)); break;
            case op_iconst_4: ir.assign(ir_value(4), allocate_stack(locals_count, stack_size)); break;
            case op_iconst_5: ir.assign(ir_value(5), allocate_stack(locals_count, stack_size)); break;
            case op_ldc: {
                read_byte1(index)
                auto& entry = current_class->constant_pool[index - 1];
                switch (entry.tag) {
                    case constant_pool_tag::CONSTANT_Integer: {
                        auto value = *reinterpret_cast<jvm_int*>(entry.info);
                        ir.assign(ir_value(value), allocate_stack(locals_count, stack_size));
                        break;
                    }
                    default:
                        throw runtime_error("Unsupported ldc parameter");
                }
                break;
            }
            case op_bipush: {
                read_byte1(value)
                ir.assign(ir_value((int8_t) value), allocate_stack(locals_count, stack_size));
                break;
            }
            case op_sipush: {
                read_byte2(value)
                ir.assign(ir_value((int16_t) value), allocate_stack(locals_count, stack_size));
                break;
            }
            case op_iload: {
                read_byte1(index)
                ir.assign(get_local(index), allocate_stack(locals_count, stack_size));
                break;
            }
            case op_iload_0: ir.assign(get_local(0), allocate_stack(locals_count, stack_size)); break;
            case op_iload_1: ir.assign(get_local(1), allocate_stack(locals_count, stack_size)); break;
            case op_iload_2: ir.assign(get_local(2), allocate_stack(locals_count, stack_size)); break;
            case op_iload_3: ir.assign(get_local(3), allocate_stack(locals_count, stack_size)); break;
            case op_istore: {
                read_byte1(index)
                ir.assign(pop_stack(locals_count, stack_size), get_local(index));
                break;
            }
            case op_istore_0: ir.assign(pop_stack(locals_count, stack_size), get_local(0)); break;
            case op_istore_1: ir.assign(pop_stack(locals_count, stack_size), get_local(1)); break;
            case op_istore_2: ir.assign(pop_stack(locals_count, stack_size), get_local(2)); break;
            case op_istore_3: ir.assign(pop_stack(locals_count, stack_size), get_local(3)); break;
            case op_iadd:
            case op_isub:
            case op_imul:
            case op_idiv:
            case op_irem: {
                auto var2 = pop_stack(locals_count, stack_size);
                auto var1 = pop_stack(locals_count, stack_size);
                auto result = allocate_stack(locals_count, stack_size);
                ir_bin_op op;
                switch (code[offset]) {
                    case op_iadd: op = ir_bin_op::add; break;
                    case op_isub: op = ir_bin_op::sub; break;
                    case op_imul: op = ir_bin_op::mul; break;
                    case op_idiv: op = ir_bin_op::div; break;
                    case op_irem: op = ir_bin_op::rem; break;
                    default: assert(false)
                }
                ir.bin_op(var1, var2, result, op);
                break;
            }
            case op_iinc: {
                read_byte1(index)
                read_byte1(param)
                auto var = get_local(index);
                ir.bin_op(var, ir_value(param), var, ir_bin_op::add);
                break;
            }
            case op_if_icmpeq:
            case op_if_icmpne:
            case op_if_icmplt:
            case op_if_icmpge:
            case op_if_icmpgt:
            case op_if_icmple: {
                uint8_t branchbyte1 = code[offset + 1];
                uint8_t branchbyte2 = code[offset + 2];
                int16_t bytecode_offset = (branchbyte1 << 8) | branchbyte2;
                auto var2 = pop_stack(locals_count, stack_size);
                auto var1 = pop_stack(locals_count, stack_size);
                auto target_offset = offset + bytecode_offset;
                auto iter = bytecode_offset_to_ir.find(target_offset);
                ir_label label_true = ir.create_label();
                ir_label label_false = ir.create_label();
                if (iter != bytecode_offset_to_ir.end()) {
                    ir.add_label(label_true, iter->second);
                } else {
                    pending_labels.insert_or_assign(target_offset, label_true);
                }
                ir_cmp_mode cmp_mode;
                switch (code[offset]) {
                    case op_if_icmpeq: cmp_mode = ir_cmp_mode::eq; break;
                    case op_if_icmpne: cmp_mode = ir_cmp_mode::neq; break;
                    case op_if_icmplt: cmp_mode = ir_cmp_mode::lt; break;
                    case op_if_icmpge: cmp_mode = ir_cmp_mode::ge; break;
                    case op_if_icmpgt: cmp_mode = ir_cmp_mode::gt; break;
                    case op_if_icmple: cmp_mode = ir_cmp_mode::le; break;
                    default: assert(false)
                }
                ir.cmp_jump(var1, var2, cmp_mode, label_true, label_false);
                ir.add_label(label_false, ir.ir_offset());
                offset += 2;
                break;
            }
            case op_ifeq:
            case op_ifne:
            case op_iflt:
            case op_ifle:
            case op_ifgt:
            case op_ifge: {
                uint8_t branchbyte1 = code[offset + 1];
                uint8_t branchbyte2 = code[offset + 2];
                int16_t bytecode_offset = (branchbyte1 << 8) | branchbyte2;
                auto var = pop_stack(locals_count, stack_size);
                auto target_offset = offset + bytecode_offset;
                auto iter = bytecode_offset_to_ir.find(target_offset);
                ir_label label_true = ir.create_label();
                ir_label label_false = ir.create_label();
                if (iter != bytecode_offset_to_ir.end()) {
                    ir.add_label(label_true, iter->second);
                } else {
                    pending_labels.insert_or_assign(target_offset, label_true);
                }
                ir_cmp_mode cmp_mode;
                switch (code[offset]) {
                    case op_ifeq: cmp_mode = ir_cmp_mode::eq; break;
                    case op_ifne: cmp_mode = ir_cmp_mode::neq; break;
                    case op_iflt: cmp_mode = ir_cmp_mode::lt; break;
                    case op_ifge: cmp_mode = ir_cmp_mode::ge; break;
                    case op_ifgt: cmp_mode = ir_cmp_mode::gt; break;
                    case op_ifle: cmp_mode = ir_cmp_mode::le; break;
                    default: assert(false)
                }
                ir.cmp_jump(var, ir_value(0), cmp_mode, label_true, label_false);
                ir.add_label(label_false, ir.ir_offset());
                offset += 2;
                break;
            }
            case op_goto: {
                uint8_t branchbyte1 = code[offset + 1];
                uint8_t branchbyte2 = code[offset + 2];
                int16_t bytecode_offset = (branchbyte1 << 8) | branchbyte2;
                auto target_offset = offset + bytecode_offset;
                auto iter = bytecode_offset_to_ir.find(target_offset);
                ir_label label_true = ir.create_label();
                if (iter != bytecode_offset_to_ir.end()) {
                    ir.add_label(label_true, iter->second);
                } else {
                    pending_labels.insert_or_assign(target_offset, label_true);
                }
                ir.jump(label_true);
                offset += 2;
                break;
            }
            case op_ireturn: {
                ir.ret(pop_stack(locals_count, stack_size));
                break;
            }
            default:
                printf("Unknown opcode: 0x%02x\n", code[offset]);
                assert(false)
        }
        offset++;
    }

    ir.pretty_print(cout);
    return ir.compile();
}
