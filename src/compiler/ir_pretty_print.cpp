#include "ir_compiler.h"
#include "../utils/utils.h"
#include <ostream>

using namespace std;

ostream& operator<<(ostream& stream, const ir_variable& var) {
    return stream << "var" << var.id << "." << var.version;
}

ostream& operator<<(ostream& stream, const ir_value& var) {
    switch (var.mode) {
        case ir_value_mode::var:
            stream << var.var;
            break;
        case ir_value_mode::int64:
            stream << var.value;
            break;
        default:
            assert(false)
    }
    return stream;
}

ostream& operator<<(ostream& stream, const shared_ptr<ir_instruction>& item) {
    stream << "    ";
    switch (item->tag) {
        case ir_instruction_tag::assign: {
            auto instruction = static_pointer_cast<ir_assign_instruction>(item);
            stream << instruction->to << " = " << instruction->from << endl;
            break;
        }
        case ir_instruction_tag::bin_op: {
            auto instruction = static_pointer_cast<ir_bin_op_insruction>(item);
            stream << instruction->to << " = " << instruction->first;
            switch (instruction->op) {
                case ir_bin_op::add: stream << " + "; break;
                case ir_bin_op::sub: stream << " - "; break;
                case ir_bin_op::mul: stream << " * "; break;
                case ir_bin_op::div: stream << " / "; break;
                case ir_bin_op::rem: stream << " % "; break;
                case ir_bin_op::cmp: stream << " compare "; break;
                default: assert(false)
            }
            stream << instruction->second << endl;
            break;
        }
        case ir_instruction_tag::convert: {
            auto instruction = static_pointer_cast<ir_convert_instruction>(item);
            stream << instruction->to << " = ";
            switch (instruction->mode) {
                case ir_convert_mode::i2l: stream << "(long) "; break;
                case ir_convert_mode::l2i: stream << "(int) "; break;
                default_fail
            }
            stream << instruction->from << endl;
            break;
        }
        case ir_instruction_tag::cmp_jump: {
            auto instruction = static_pointer_cast<ir_cmp_jump_instruction>(item);
            stream << "if (" << instruction->first;
            switch (instruction->mode) {
                case ir_cmp_mode::eq:  stream << " == "; break;
                case ir_cmp_mode::neq: stream << " != "; break;
                case ir_cmp_mode::lt:  stream << " < ";  break;
                case ir_cmp_mode::le:  stream << " <= "; break;
                case ir_cmp_mode::gt:  stream << " > ";  break;
                case ir_cmp_mode::ge:  stream << " >= "; break;
                default: assert(false);
            }
            stream << instruction->second << ") then goto L" << instruction->label_true.id
                   << " else goto L" << instruction->label_false.id << endl;
            break;
        }
        case ir_instruction_tag::jump: {
            auto instruction = static_pointer_cast<ir_jump_instruction>(item);
            stream << "goto L" << instruction->label.id << endl;
            break;
        }
        case ir_instruction_tag::ret: {
            auto instruction = static_pointer_cast<ir_ret_instruction>(item);
            stream << "return " << instruction->value << endl;
            break;
        }
        case ir_instruction_tag::phi: {
            auto instruction = static_pointer_cast<ir_phi_instruction>(item);
            stream << instruction->to << " = phi(";
            for (int i = 0; i < instruction->edges.size(); i++) {
                const auto& edge = instruction->edges[i];
                stream << "[L" << edge.first.id << ", " << edge.second << "]";
                if (i < instruction->edges.size() - 1) {
                    stream << ", ";
                }
            }
            stream << ")" << endl;
            break;
        }
        default:
            assert(false)
    }
    return stream;
}

void ir_compiler::pretty_print(ostream& stream) {
    cout << "---- Converted IR code ----" << endl;
    for (int ir_offset = 0; ir_offset < ir.size(); ir_offset++) {
        const auto& item = ir[ir_offset];
        auto label_iter = offset_to_label.find(ir_offset);
        if (label_iter != offset_to_label.end()) {
            stream << "L" << label_iter->second.id << ":" << endl;
        }
        stream << item;
    }
}

void ir_compiler::pretty_print(ostream& stream, const ir_basic_block& block) {
    stream << "L" << block.label.id << ":" << endl;
    for (const auto& item : block.ir) {
        stream << item;
    }
}