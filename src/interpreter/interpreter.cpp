#include "../vm/vm.h"
#include "opcodes.h"
#include "../utils/utils.h"
#include "../utils/constant_pool_utils.h"
#include "../vm/object_intrinsics.h"
#include "../vm/jvm_defines.h"
#include "../vm/object_intrinsics.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>

using namespace std;

#define pop1(x) auto x = stack.back(); stack.pop_back();
#define pop2(x, y) auto y = stack.back(); stack.pop_back(); auto x = stack.back(); stack.pop_back();
#define pop3(x, y, z) auto z = stack.back(); stack.pop_back(); auto y = stack.back(); stack.pop_back(); auto x = stack.back(); stack.pop_back();
#define push(x) stack.push_back(x);
#define push_byte(x) stack.push_back(jvm_value::as_byte(x));
#define push_short(x) stack.push_back(jvm_value::as_short(x));
#define push_int(x) stack.push_back(jvm_value::as_int(x));
#define push_long(x) stack.push_back(jvm_value::as_int(x));
#define push_float(x) stack.push_back(jvm_value::as_float(x));
#define push_double(x) stack.push_back(jvm_value::as_double(x));
#define push_boolean(x) stack.push_back(jvm_value::as_boolean(x));
#define push_char(x) stack.push_back(jvm_value::as_char(x));
#define push_reference(x) stack.push_back(jvm_value::as_reference(x));
#define int_bin_op(x, op, y) stack.push_back(jvm_value::as_int((x).int_value op (y).int_value));
#define float_bin_op(x, op, y) stack.push_back(jvm_value::as_int((x).float_value op (y).float_value));
#define double_bin_op(x, op, y) stack.push_back(jvm_value::as_double((x).double_value op (y).double_value));
#define read_byte1(name) uint8_t name = code[++offset];
#define read_byte2(name) uint8_t name##byte1 = code[++offset]; uint8_t name##byte2 = code[++offset]; uint16_t name = (name##byte1 << 8) | name##byte2;
#define int_cmp_zero_op(op) {\
        uint8_t branchbyte1 = code[offset + 1]; \
        uint8_t branchbyte2 = code[offset + 2]; \
        int16_t branchoffset = (branchbyte1 << 8) | branchbyte2; \
        pop1(op1) \
        if (op1.int_value op 0) { \
            offset += branchoffset; \
            goto skip_offset_increment; \
        } else { \
            offset += 2; \
        } \
        break; \
    }
#define int_cmp_op(op) {\
        uint8_t branchbyte1 = code[offset + 1]; \
        uint8_t branchbyte2 = code[offset + 2]; \
        int16_t branchoffset = (branchbyte1 << 8) | branchbyte2; \
        pop2(op1, op2) \
        if (op1.int_value op op2.int_value) { \
            offset += branchoffset; \
            goto skip_offset_increment; \
        } else { \
            offset += 2; \
        } \
        break; \
    }
#define ref_cmp_null_op(op) {\
        uint8_t branchbyte1 = code[offset + 1]; \
        uint8_t branchbyte2 = code[offset + 2]; \
        int16_t branchoffset = (branchbyte1 << 8) | branchbyte2; \
        pop1(op1) \
        if (op1.reference_value op) { \
            offset += branchoffset; \
            goto skip_offset_increment; \
        } else { \
            offset += 2; \
        } \
        break; \
    }

jvm_value vm::interpret(const class_file* current_class, const method_info& method, const std::vector<jvm_value>&& parameters) {
    auto code_info = get_code_info(current_class, method);
    return interpret(current_class, code_info, move(parameters));
}

jvm_value vm::interpret(const class_file* current_class, const code_attribute_info* code_info, const std::vector<jvm_value>&& parameters) {
    vector<jvm_value> stack;
    vector<jvm_value> locals;
    stack.reserve(code_info->max_stack);
    locals.resize(code_info->max_locals);

    for (int i = 0; i < parameters.size(); i++) {
        locals[i] = parameters[i];
    }

    const uint8_t* code = code_info->code;
    uint32_t offset = 0;
    while (offset < code_info->code_length) {
        switch (code[offset]) {
            case op_nop:
                break;
            case op_aconst_null: {
                push_reference(heap.null_ref())
                break;
            }
            case op_iconst_m1:
                push_int(-1)
                break;
            case op_iconst_0:
                push_int(0)
                break;
            case op_iconst_1:
                push_int(1)
                break;
            case op_iconst_2:
                push_int(2)
                break;
            case op_iconst_3:
                push_int(3)
                break;
            case op_iconst_4:
                push_int(4)
                break;
            case op_iconst_5:
                push_int(5)
                break;
            case op_fconst_0:
                push_float(0.0f)
                break;
            case op_fconst_1:
                push_float(1.0f)
                break;
            case op_fconst_2:
                push_float(2.0f)
                break;
            case op_bipush: {
                read_byte1(value)
                push_int((int8_t) value)
                break;
            }
            case op_sipush: {
                read_byte2(value)
                push_short((int16_t) value)
                break;
            }
            case op_ldc: {
                read_byte1(index)
                auto& entry = current_class->constant_pool[index - 1];
                switch (entry.tag) {
                    case constant_pool_tag::CONSTANT_Integer: {
                        auto value = *reinterpret_cast<jvm_int*>(entry.info);
                        push_int(value)
                        break;
                    }
                    case constant_pool_tag::CONSTANT_Float: {
                        auto value = *reinterpret_cast<jvm_float*>(entry.info);
                        push_float(value)
                        break;
                    }
                    case constant_pool_tag::CONSTANT_String: {
                        auto info = reinterpret_cast<cp_string*>(entry.info);
                        auto string = read_utf8_string(current_class, info->string_index);
                        auto reference = create_interned_string_instance(this, string);
                        push_reference(reference)
                        break;
                    }
                    case constant_pool_tag::CONSTANT_Class: {
                        assert(false)
                    }
                    default:
                        throw runtime_error("Unsupported ldc parameter");
                }
                break;
            }
            case op_ldc2_w: {
                read_byte2(index)
                double value = read_double(current_class, index);
                push_double(value)
                break;
            }
            case op_iload_0:
            case op_fload_0:
            case op_dload_0:
            case op_aload_0:
                push(locals[0])
                break;
            case op_iload_1:
            case op_fload_1:
            case op_dload_1:
            case op_aload_1:
                push(locals[1])
                break;
            case op_iload_2:
            case op_fload_2:
            case op_dload_2:
            case op_aload_2:
                push(locals[2])
                break;
            case op_iload_3:
            case op_fload_3:
            case op_dload_3:
            case op_aload_3:
                push(locals[3])
                break;
            case op_iaload: {
                pop2(array_ref, index)
                auto pointer = array_element<jvm_int>(array_ref.reference_value, index.int_value);
                push_int(*pointer)
                break;
            }
            case op_caload: {
                pop2(array_ref, index)
                auto pointer = array_element<jvm_char>(array_ref.reference_value, index.int_value);
                push_char(*pointer)
                break;
            }
            case op_aaload: {
                pop2(array_ref, index)
                auto pointer = array_element<jvm_reference>(array_ref.reference_value, index.int_value);
                push_reference(*pointer)
                break;
            }
            case op_iload:
            case op_dload: {
                read_byte1(index)
                push(locals[index])
                break;
            }
            case op_istore_0:
            case op_dstore_0:
            case op_astore_0:
                locals[0] = stack.back();
                stack.pop_back();
                break;
            case op_istore_1:
            case op_dstore_1:
            case op_astore_1:
                locals[1] = stack.back();
                stack.pop_back();
                break;
            case op_istore_2:
            case op_dstore_2:
            case op_astore_2:
                locals[2] = stack.back();
                stack.pop_back();
                break;
            case op_istore_3:
            case op_dstore_3:
            case op_astore_3:
                locals[3] = stack.back();
                stack.pop_back();
                break;
            case op_iastore: {
                pop3(array_ref, index, value)
                auto pointer = array_element<jvm_int>(array_ref.reference_value, index.int_value);
                *pointer = value.int_value;
                break;
            }
            case op_aastore: {
                pop3(array_ref, index, value)
                auto pointer = array_element<jvm_reference>(array_ref.reference_value, index.int_value);
                *pointer = value.reference_value;
                break;
            }
            case op_castore: {
                pop3(array_ref, index, value)
                auto pointer = array_element<jvm_char>(array_ref.reference_value, index.int_value);
                *pointer = value.char_value;
                break;
            }
            case op_istore:
            case op_dstore: {
                read_byte1(index)
                locals[index] = stack.back();
                stack.pop_back();
                break;
            }
            case op_iinc: {
                read_byte1(index)
                read_byte1(param)
                locals[index].int_value += param;
                break;
            }
            case op_i2l: { pop1(op1) push_long(op1.int_value) break; }
            case op_i2f: { pop1(op1) push_float(op1.int_value) break; }
            case op_i2d: { pop1(op1) push_double(op1.int_value) break; }
            case op_l2i: { pop1(op1) push_int(op1.long_value) break; }
            case op_l2f: { pop1(op1) push_float(op1.long_value) break; }
            case op_l2d: { pop1(op1) push_double(op1.long_value) break; }
            case op_f2i: { pop1(op1) push_int(op1.float_value) break; }
            case op_f2l: { pop1(op1) push_long(op1.float_value) break; }
            case op_f2d: { pop1(op1) push_double(op1.float_value) break; }
            case op_d2i: { pop1(op1) push_int(op1.double_value) break; }
            case op_d2l: { pop1(op1) push_long(op1.double_value) break; }
            case op_d2f: { pop1(op1) push_float(op1.double_value) break; }
            case op_i2b: { pop1(op1) push_byte(op1.int_value) break; }
            case op_i2c: { pop1(op1) push_char(op1.int_value) break; }
            case op_i2s: { pop1(op1) push_short(op1.int_value) break; }
            case op_pop: {
                stack.pop_back();
                break;
            }
            case op_pop2: {
                stack.pop_back();
                stack.pop_back();
                break;
            }
            case op_dup: {
                stack.push_back(stack.back());
                break;
            }
            case op_iadd: {
                pop2(op1, op2)
                int_bin_op(op1, +, op2)
                break;
            }
            case op_isub: {
                pop2(op1, op2)
                int_bin_op(op1, -, op2)
                break;
            }
            case op_idiv: {
                pop2(op1, op2)
                int_bin_op(op1, /, op2)
                break;
            }
            case op_dadd: {
                pop2(op1, op2)
                double_bin_op(op1, +, op2)
                break;
            }
            case op_dsub: {
                pop2(op1, op2)
                double_bin_op(op1, -, op2)
                break;
            }
            case op_imul: {
                pop2(op1, op2)
                int_bin_op(op1, *, op2)
                break;
            }
            case op_fmul: {
                pop2(op1, op2)
                float_bin_op(op1, *, op2)
                break;
            }
            case op_ddiv: {
                pop2(op1, op2)
                double_bin_op(op1, /, op2)
                break;
            }
            case op_ishl: {
                pop2(op1, op2)
                int32_t result = op1.int_value << op2.int_value;
                push_int(result)
                break;
            }
            case op_ishr: {
                pop2(op1, op2)
                int32_t result = op1.int_value >> op2.int_value;
                push_int(result)
                break;
            }
            case op_iushr: {
                pop2(op1, op2)
                int32_t result = (uint32_t) op1.int_value >> op2.int_value;
                push_int(result)
                break;
            }
            case op_ireturn:
            case op_dreturn:
            case op_areturn:
                return stack.back();
            case op_return:
                return jvm_value();
            case op_fcmpl:
            case op_fcmpg: {
                pop2(op1, op2)
                auto opcode = code[offset];
                if (isnan(op1.float_value) || isnan(op2.float_value)) {
                    push_int(opcode == op_fcmpg ? 1 : -1)
                    break;
                }

                jvm_int result = op1.float_value > op2.float_value ? 1
                        : (op1.float_value < op2.float_value ? -1 : 0);
                push_int(result)
                break;
            }
            case op_ifeq: int_cmp_zero_op(==)
            case op_ifne: int_cmp_zero_op(!=)
            case op_iflt: int_cmp_zero_op(<)
            case op_ifle: int_cmp_zero_op(<=)
            case op_ifgt: int_cmp_zero_op(>)
            case op_ifge: int_cmp_zero_op(>=)
            case op_if_icmpeq: int_cmp_op(==)
            case op_if_icmpne: int_cmp_op(!=)
            case op_if_icmplt: int_cmp_op(<)
            case op_if_icmple: int_cmp_op(<=)
            case op_if_icmpgt: int_cmp_op(>)
            case op_if_icmpge: int_cmp_op(>=)
            case op_goto: {
                uint8_t branchbyte1 = code[offset + 1];
                uint8_t branchbyte2 = code[offset + 2];
                int16_t branchoffset = (branchbyte1 << 8) | branchbyte2;
                offset += branchoffset;
                goto skip_offset_increment;
            }
            case op_getstatic: {
                read_byte2(index)
                auto [field_reference, bytes_to_copy] = get_static_field_info(current_class, index);
                jvm_value result;
                memcpy(&result, field_reference, bytes_to_copy);
                push(result)
                break;
            }
            case op_putstatic: {
                read_byte2(index)
                pop1(value_v)
                auto [field_reference, bytes_to_copy] = get_static_field_info(current_class, index);
                memcpy(field_reference, &value_v, bytes_to_copy);
                break;
            }
            case op_getfield: {
                read_byte2(index)
                pop1(object_v)
                auto [field_offset, bytes_to_copy] = get_field_info(current_class, index);
                jvm_value result;
                memcpy(&result, object_v.reference_value + field_offset, bytes_to_copy);
                push(result)
                break;
            }
            case op_putfield: {
                read_byte2(index)
                pop2(object_v, value_v)
                auto [field_offset, bytes_to_copy] = get_field_info(current_class, index);
                memcpy(object_v.reference_value + field_offset, &value_v, bytes_to_copy);
                break;
            }
            case op_invokevirtual: // for now all three implementations are the same
            case op_invokespecial:
            case op_invokestatic: {
                bool is_instance_invoke = code[offset] == op_invokevirtual || code[offset] == op_invokespecial;
                read_byte2(index)
                // TODO value set conversion (§2.8.3)
                const auto* method_ref = read_method_ref(current_class, index);

                // TODO for now check java/lang/Object.<init> and skip it
                if (is_java_lang_object_init(current_class, method_ref)) {
                    stack.pop_back();
                    break;
                }

                const auto [class_info, method_code_info] = get_code_info(current_class, method_ref);
                auto method_descriptor_string = read_utf8_string(current_class, read_name_and_type(current_class, method_ref->name_and_type_index)->descriptor_index);
                auto descriptor = method_descriptor::parse(method_descriptor_string);
                auto class_name = read_utf8_string(current_class, read_class_info(current_class, method_ref->class_index)->name_index);
                if (is_instance_invoke) {
                    descriptor.parameters.insert(descriptor.parameters.begin(), field_descriptor::parse("L" + class_name + ";"));
                }

                // TODO remove method name, for debug only
                auto method_name = read_utf8_string(current_class, read_name_and_type(current_class, method_ref->name_and_type_index)->name_index);
                invoke_with_parameters(stack, descriptor, class_info, method_code_info);
                break;
            }
            case op_new: {
                read_byte2(index)
                const auto* class_info = read_class_info(current_class, index);
                const auto& target_class = get_or_load_class(current_class, class_info);
                jvm_reference new_reference = heap.allocate(target_class->total_object_size);
                push_reference(new_reference)
                break;
            }
            case op_newarray: {
                read_byte1(a_type)
                pop1(count)
                auto new_reference = create_array_instance(this, count.int_value, atype_size(static_cast<atype>(a_type)));
                push_reference(new_reference)
                break;
            }
            case op_anewarray: {
                read_byte2(index)
                pop1(count)
                auto new_reference = create_array_instance(this, count.int_value, sizeof(jvm_reference));
                push_reference(new_reference)
                break;
            }
            case op_arraylength: {
                pop1(array_ref)
                jvm_int array_size = array_length(array_ref.reference_value);
                push_int(array_size)
                break;
            }
            case op_monitorenter: {
                pop1(object_ref)
                auto& mutex = monitor_map[object_ref.reference_value];
                mutex.lock();
                break;
            }
            case op_monitorexit: {
                pop1(object_ref)
                auto& mutex = monitor_map[object_ref.reference_value];
                mutex.unlock();
                break;
            }
            case op_ifnull: ref_cmp_null_op(== heap.null_ref())
            case op_ifnonnull: ref_cmp_null_op(!= heap.null_ref())
            default:
                printf("Unknown opcode: 0x%02x\n", code[offset]);
                assert(false)
        }
        offset++;
        skip_offset_increment:;
    }
    assert(false)
}

pair<uint64_t, uint8_t> get_field_info_impl(vm* jvm, const class_file* current_class, uint16_t index, bool is_static) {
    auto field_ref = read_field_ref(current_class, index);
    auto class_info = read_class_info(current_class, field_ref->class_index);
    auto class_file = jvm->get_or_load_class(current_class, class_info);
    auto name_and_type = read_name_and_type(current_class, field_ref->name_and_type_index);
    auto field_name = read_utf8_string(current_class, name_and_type->name_index);
    uint64_t field_offset;
    if (is_static) {
        auto field_offset_it = class_file->static_fields.find(field_name);
        assert(field_offset_it != class_file->static_fields.end())
        field_offset = reinterpret_cast<uint64_t>(field_offset_it->second);
    } else {
        auto field_offset_it = class_file->field_offsets.find(field_name);
        assert(field_offset_it != class_file->field_offsets.end())
        field_offset = field_offset_it->second;
    }
    auto descriptor_string = read_utf8_string(current_class, name_and_type->descriptor_index);
    auto descriptor = field_descriptor::parse(descriptor_string);
    return make_pair(field_offset, descriptor.get_field_size());
}

pair<uint64_t, uint8_t> vm::get_field_info(const class_file* current_class, uint16_t index) {
    return get_field_info_impl(this, current_class, index, false);
}

pair<jvm_reference, uint8_t> vm::get_static_field_info(const class_file* current_class, uint16_t index) {
    auto [field_offset, field_size] = get_field_info_impl(this, current_class, index, true);
    return make_pair(reinterpret_cast<jvm_reference>(field_offset), field_size);
}

void vm::invoke_with_parameters(vector<jvm_value>& stack, const method_descriptor& descriptor,
                                const class_file* class_info, const code_attribute_info* method_code_info) {
    vector<jvm_value> invoke_parameters;
    int local_offset = 0;
    int stack_top_index = descriptor.parameters.size();
    for (const auto& parameter: descriptor.parameters) {
        const auto& base_type = parameter.get_base_type();
        int param_size = 1;
        if (base_type == base_type_descriptor::long_d || base_type == base_type_descriptor::double_d) {
            param_size = 2;
        }
        invoke_parameters.resize(invoke_parameters.size() + param_size);
        invoke_parameters[local_offset] = stack[stack.size() - stack_top_index];
        local_offset += param_size;
        stack_top_index--;
    }
    stack.resize(stack.size() - descriptor.parameters.size());

    // TODO implement natives
    if (method_code_info != nullptr) {
        jvm_value result = interpret(class_info, method_code_info, move(invoke_parameters));
        if (!descriptor.return_void) {
            stack.push_back(result);
        }
    } else {
        if (!descriptor.return_void) {
            stack.push_back(jvm_value());
        }
    }
}