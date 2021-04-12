#include "ir_compiler.h"
#include "../utils/utils.h"
#include <memory>

using namespace std;

void ir_compiler::assign(ir_value from, ir_variable to) {
    assert(from.mode != ir_value_mode::var || from.var.type == to.type)
    ir.push_back(make_shared<ir_assign_instruction>(from, to));
}

void ir_compiler::bin_op(ir_value first, ir_value second, ir_variable to, ir_bin_op op, ir_cmp_nan_mode nan_mode) {
    ir.push_back(make_shared<ir_bin_op_insruction>(first, second, to, op, nan_mode));
}

void ir_compiler::convert(ir_value from, ir_variable to, ir_convert_mode mode) {
    ir.push_back(make_shared<ir_convert_instruction>(from, to, mode));
}

void ir_compiler::cmp_jump(ir_value first, ir_value second, ir_cmp_mode mode, ir_label label_true, ir_label label_false) {
    ir.push_back(make_shared<ir_cmp_jump_instruction>(first, second, mode, label_true, label_false));
}

void ir_compiler::jump(ir_label label) {
    ir.push_back(make_shared<ir_jump_instruction>(label));
}

void ir_compiler::ret(ir_value value) {
    ir.push_back(make_shared<ir_ret_instruction>(value));
}

void ir_compiler::phi(const std::vector<std::pair<ir_label, ir_value>>& edges, const ir_variable& to) {
    ir.push_back(make_shared<ir_phi_instruction>(edges, to));
}