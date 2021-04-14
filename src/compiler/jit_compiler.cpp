#include "jit_compiler.h"
#include "../utils/constant_pool_utils.h"
#include "../interpreter/method_descriptor.h"
#include "../utils/utils.h"
#include "../interpreter/opcodes.h"
#include "register_shortcuts.h"
#include "ir_compiler.h"
#include <iostream>

using namespace std;

constexpr auto ir_int = ir_variable_type::ir_int;
constexpr auto ir_long = ir_variable_type::ir_long;
constexpr auto ir_double = ir_variable_type::ir_double;

#define read_byte1(name) uint8_t name = code[++offset];
#define read_byte2(name) uint8_t name##byte1 = code[++offset]; uint8_t name##byte2 = code[++offset]; uint16_t name = (name##byte1 << 8) | name##byte2;

jit_local_state::jit_local_state(int locals_count, int max_stack_size) {
    locals.resize(locals_count, ir_variable_type::ir_no_type);
    this->max_stack_size = max_stack_size;
}

ir_variable jit_local_state::allocate_stack(ir_variable_type type) {
    auto type_id = static_cast<int>(type);
    int stack_index = stack.size();
    stack.push_back(type);
    return ir_variable((locals.size() + max_stack_size) * type_id + locals.size() + stack_index, type);
}

ir_variable jit_local_state::pop_stack(ir_variable_type expected_type) {
    auto type = stack.back();
    assert(type == expected_type)
    auto type_id = static_cast<int>(type);
    stack.pop_back();
    int stack_index = stack.size();
    return ir_variable((locals.size() + max_stack_size) * type_id + locals.size() + stack_index, type);
}

ir_variable jit_local_state::get_local_read(int index, ir_variable_type expected_type) {
    assert(locals[index] == expected_type)
    auto type_id = static_cast<int>(locals[index]);
    return ir_variable((locals.size() + max_stack_size) * type_id + index, locals[index]);
}

ir_variable jit_local_state::get_local_write(int index, ir_variable_type new_type) {
    locals[index] = new_type;
    auto type_id = static_cast<int>(locals[index]);
    return ir_variable((locals.size() + max_stack_size) * type_id + index, locals[index]);
}

ir_variable_type to_ir_type(const field_descriptor& descriptor) {
    switch (descriptor.get_base_type()) {
        case base_type_descriptor::reference_d: assert(false)
        case base_type_descriptor::byte_d: assert(false)
        case base_type_descriptor::char_d: assert(false)
        case base_type_descriptor::double_d: return ir_double;
        case base_type_descriptor::float_d: assert(false)
        case base_type_descriptor::int_d: return ir_int;
        case base_type_descriptor::long_d: return ir_long;
        case base_type_descriptor::short_d: assert(false)
        case base_type_descriptor::boolean_d: assert(false)
        default_fail
    }
}

const void* jit_compiler::compile(const class_file* current_class, const method_info& method, const code_attribute_info* code_info) {
    auto descriptor_string = read_utf8_string(current_class, method.descriptor_index);
    auto descriptor = method_descriptor::parse(descriptor_string);

    jit_local_state local_state(code_info->max_locals, code_info->max_stack);

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
            case op_iconst_m1: ir.assign(ir_value(-1LL), local_state.allocate_stack(ir_int)); break;
            case op_iconst_0: ir.assign(ir_value(0LL), local_state.allocate_stack(ir_int)); break;
            case op_iconst_1: ir.assign(ir_value(1LL), local_state.allocate_stack(ir_int)); break;
            case op_iconst_2: ir.assign(ir_value(2LL), local_state.allocate_stack(ir_int)); break;
            case op_iconst_3: ir.assign(ir_value(3LL), local_state.allocate_stack(ir_int)); break;
            case op_iconst_4: ir.assign(ir_value(4LL), local_state.allocate_stack(ir_int)); break;
            case op_iconst_5: ir.assign(ir_value(5LL), local_state.allocate_stack(ir_int)); break;
            case op_ldc: {
                read_byte1(index)
                auto& entry = current_class->constant_pool[index - 1];
                switch (entry.tag) {
                    case constant_pool_tag::CONSTANT_Integer: {
                        auto value = *reinterpret_cast<jvm_int*>(entry.info);
                        // TODO add int32 mode
                        ir.assign(ir_value((int64_t) value), local_state.allocate_stack(ir_int));
                        break;
                    }
                    default:
                        throw runtime_error("Unsupported ldc parameter");
                }
                break;
            }
            case op_ldc2_w: {
                read_byte2(index)
                auto& entry = current_class->constant_pool[index - 1];
                switch (entry.tag) {
                    case constant_pool_tag::CONSTANT_Long: {
                        auto value = *reinterpret_cast<jvm_long*>(entry.info);
                        ir.assign(ir_value(value), local_state.allocate_stack(ir_long));
                        break;
                    }
                    case constant_pool_tag::CONSTANT_Double: {
                        auto value = *reinterpret_cast<jvm_double*>(entry.info);
                        ir.assign(ir_value(value), local_state.allocate_stack(ir_double));
                        break;
                    }
                    default:
                        throw runtime_error("Unsupported ldc2_w parameter");
                }
                break;
            }
            case op_bipush: {
                read_byte1(value)
                // TODO add int32 mode
                ir.assign(ir_value((int64_t) (int8_t) value), local_state.allocate_stack(ir_int));
                break;
            }
            case op_sipush: {
                read_byte2(value)
                // TODO add int32 mode
                ir.assign(ir_value((int64_t) (int16_t) value), local_state.allocate_stack(ir_int));
                break;
            }
            case op_iload: {
                read_byte1(index)
                ir.assign(local_state.get_local_read(index, ir_int), local_state.allocate_stack(ir_int));
                break;
            }
            case op_dload: {
                read_byte1(index)
                ir.assign(local_state.get_local_read(index, ir_double), local_state.allocate_stack(ir_double));
                break;
            }
            case op_iload_0: ir.assign(local_state.get_local_read(0, ir_int), local_state.allocate_stack(ir_int)); break;
            case op_iload_1: ir.assign(local_state.get_local_read(1, ir_int), local_state.allocate_stack(ir_int)); break;
            case op_iload_2: ir.assign(local_state.get_local_read(2, ir_int), local_state.allocate_stack(ir_int)); break;
            case op_iload_3: ir.assign(local_state.get_local_read(3, ir_int), local_state.allocate_stack(ir_int)); break;
            case op_lload_0: ir.assign(local_state.get_local_read(0, ir_long), local_state.allocate_stack(ir_long)); break;
            case op_lload_1: ir.assign(local_state.get_local_read(1, ir_long), local_state.allocate_stack(ir_long)); break;
            case op_lload_2: ir.assign(local_state.get_local_read(2, ir_long), local_state.allocate_stack(ir_long)); break;
            case op_lload_3: ir.assign(local_state.get_local_read(3, ir_long), local_state.allocate_stack(ir_long)); break;
            case op_dload_0: ir.assign(local_state.get_local_read(0, ir_double), local_state.allocate_stack(ir_double)); break;
            case op_dload_1: ir.assign(local_state.get_local_read(1, ir_double), local_state.allocate_stack(ir_double)); break;
            case op_dload_2: ir.assign(local_state.get_local_read(2, ir_double), local_state.allocate_stack(ir_double)); break;
            case op_dload_3: ir.assign(local_state.get_local_read(3, ir_double), local_state.allocate_stack(ir_double)); break;
            case op_istore: {
                read_byte1(index)
                ir.assign(local_state.pop_stack(ir_int), local_state.get_local_write(index, ir_int));
                break;
            }
            case op_dstore: {
                read_byte1(index)
                ir.assign(local_state.pop_stack(ir_double), local_state.get_local_write(index, ir_double));
                break;
            }
            case op_istore_0: ir.assign(local_state.pop_stack(ir_int), local_state.get_local_write(0, ir_int)); break;
            case op_istore_1: ir.assign(local_state.pop_stack(ir_int), local_state.get_local_write(1, ir_int)); break;
            case op_istore_2: ir.assign(local_state.pop_stack(ir_int), local_state.get_local_write(2, ir_int)); break;
            case op_istore_3: ir.assign(local_state.pop_stack(ir_int), local_state.get_local_write(3, ir_int)); break;
            case op_lstore_0: ir.assign(local_state.pop_stack(ir_long), local_state.get_local_write(0, ir_long)); break;
            case op_lstore_1: ir.assign(local_state.pop_stack(ir_long), local_state.get_local_write(1, ir_long)); break;
            case op_lstore_2: ir.assign(local_state.pop_stack(ir_long), local_state.get_local_write(2, ir_long)); break;
            case op_lstore_3: ir.assign(local_state.pop_stack(ir_long), local_state.get_local_write(3, ir_long)); break;
            case op_dstore_0: ir.assign(local_state.pop_stack(ir_double), local_state.get_local_write(0, ir_double)); break;
            case op_dstore_1: ir.assign(local_state.pop_stack(ir_double), local_state.get_local_write(1, ir_double)); break;
            case op_dstore_2: ir.assign(local_state.pop_stack(ir_double), local_state.get_local_write(2, ir_double)); break;
            case op_dstore_3: ir.assign(local_state.pop_stack(ir_double), local_state.get_local_write(3, ir_double)); break;
            case op_iadd:
            case op_isub:
            case op_imul:
            case op_idiv:
            case op_irem: {
                auto var2 = local_state.pop_stack(ir_int);
                auto var1 = local_state.pop_stack(ir_int);
                auto result = local_state.allocate_stack(ir_int);
                ir_bin_op op;
                switch (code[offset]) {
                    case op_iadd: op = ir_bin_op::add; break;
                    case op_isub: op = ir_bin_op::sub; break;
                    case op_imul: op = ir_bin_op::mul; break;
                    case op_idiv: op = ir_bin_op::div; break;
                    case op_irem: op = ir_bin_op::rem; break;
                    default_fail
                }
                ir.bin_op(var1, var2, result, op);
                break;
            }
            case op_lmul:
            case op_lrem: {
                auto var2 = local_state.pop_stack(ir_long);
                auto var1 = local_state.pop_stack(ir_long);
                auto result = local_state.allocate_stack(ir_long);
                ir_bin_op op;
                switch (code[offset]) {
                    case op_lmul: op = ir_bin_op::mul; break;
                    case op_lrem: op = ir_bin_op::rem; break;
                    default_fail
                }
                ir.bin_op(var1, var2, result, op);
                break;
            }
            case op_dadd:
            case op_dsub:
            case op_dmul: {
                auto var2 = local_state.pop_stack(ir_double);
                auto var1 = local_state.pop_stack(ir_double);
                auto result = local_state.allocate_stack(ir_double);
                ir_bin_op op;
                switch (code[offset]) {
                    case op_dadd: op = ir_bin_op::add; break;
                    case op_dsub: op = ir_bin_op::sub; break;
                    case op_dmul: op = ir_bin_op::mul; break;
                    default_fail
                }
                ir.bin_op(var1, var2, result, op);
                break;
            }
            case op_i2l: {
                auto var = local_state.pop_stack(ir_int);
                auto result = local_state.allocate_stack(ir_long);
                ir.convert(var, result, ir_convert_mode::i2l);
                break;
            }
            case op_l2i: {
                auto var = local_state.pop_stack(ir_long);
                auto result = local_state.allocate_stack(ir_int);
                ir.convert(var, result, ir_convert_mode::l2i);
                break;
            }
            case op_iinc: {
                read_byte1(index)
                read_byte1(param)
                // get_local_write is not needed here, because old and new types are the same
                auto var = local_state.get_local_read(index, ir_int);
                // TODO add int32 mode
                ir.bin_op(var, ir_value((int64_t) (int8_t) param), var, ir_bin_op::add);
                break;
            }
            case op_lcmp: {
                auto var2 = local_state.pop_stack(ir_long);
                auto var1 = local_state.pop_stack(ir_long);
                auto result = local_state.allocate_stack(ir_int);
                ir.bin_op(var1, var2, result, ir_bin_op::cmp);
                break;
            }
            case op_dcmpl:
            case op_dcmpg: {
                auto var2 = local_state.pop_stack(ir_double);
                auto var1 = local_state.pop_stack(ir_double);
                auto result = local_state.allocate_stack(ir_int);
                ir_cmp_nan_mode nan_mode;
                switch (code[offset]) {
                    case op_dcmpl: nan_mode = ir_cmp_nan_mode::neg_unit; break;
                    case op_dcmpg: nan_mode = ir_cmp_nan_mode::unit; break;
                    default_fail
                }
                ir.bin_op(var1, var2, result, ir_bin_op::cmp, nan_mode);
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
                auto var2 = local_state.pop_stack(ir_int);
                auto var1 = local_state.pop_stack(ir_int);
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
                ir.cmp_jump(var1, var2, cmp_mode, label_true, label_false, ir_cmp_nan_mode::no_nan);
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
                auto var = local_state.pop_stack(ir_int);
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
                ir.cmp_jump(var, ir_value(0LL), cmp_mode, label_true, label_false, ir_cmp_nan_mode::no_nan);
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
            case op_ireturn:
            case op_dreturn: {
                assert(!descriptor.return_void)
                auto return_type = to_ir_type(descriptor.return_descriptor);
                ir.ret(local_state.pop_stack(return_type));
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
