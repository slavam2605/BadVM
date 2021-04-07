#ifndef BADJVM_IR_COMPILER_H
#define BADJVM_IR_COMPILER_H

#include "ir_instructions.h"
#include "code_builder.h"
#include "code_manager.h"
#include <cstdint>
#include <memory>
#include <vector>
#include <unordered_map>
#include <iostream>

struct ir_basic_block {
    ir_label label;
    std::vector<std::shared_ptr<ir_instruction>> ir;

    ir_basic_block(const ir_label& label);
};

class ir_compiler {
    code_manager& manager;
    code_builder builder;
    std::vector<std::shared_ptr<ir_instruction>> ir;
    std::unordered_map<int, ir_label> offset_to_label;
    int last_label_id = 0;
    std::vector<ir_basic_block> blocks;

    // Valid after coloring
    std::unordered_map<ir_variable, std::vector<ir_variable>> color_preference;
    std::unordered_map<ir_variable, std::unordered_set<jit_register64>> reg_preference;
    std::unordered_map<ir_variable, int> color;
    std::unordered_map<ir_label, const ir_basic_block*> block_map;
    std::vector<jit_register64> reg_list;

public:
    ir_compiler(code_manager& manager);
    int ir_offset() const;
    ir_label create_label();
    void add_label(ir_label label, int ir_offset);
    void pretty_print(std::ostream& stream);
    void pretty_print(std::ostream& stream, const ir_basic_block& block);
    void compile_assign(const ir_value& from_value, const jit_register64& to);
    void compile_assign(const jit_register64& from, const jit_register64& to);
    void compile_phi_dfs(const jit_register64& start, int version,
                         const std::unordered_map<jit_register64, std::unordered_set<jit_register64>>& assign_from_map,
                         std::unordered_map<jit_register64, int>& visited_version,
                         std::unordered_map<jit_register64, jit_register64>& temp);
    void compile_phi_before_jump(const ir_label& current_label, const ir_basic_block* target_block);
    void compile_bin_op(const std::shared_ptr<ir_bin_op_insruction>& instruction);
    const uint8_t* compile_ssa();
    void calculate_color_preferences();
    void color_variables();
    void optimize();
    void convert_to_ssa();
    const uint8_t* compile();

    void assign(ir_value from, ir_variable to);
    void bin_op(ir_value first, ir_value second, ir_variable to, ir_bin_op op);
    void convert(ir_value from, ir_variable to, ir_convert_mode mode);
    void cmp_jump(ir_value first, ir_value second, ir_cmp_mode mode, ir_label label_true, ir_label label_false);
    void jump(ir_label label);
    void ret(ir_value value);
    void phi(const std::vector<std::pair<ir_label, ir_value>>& edges, const ir_variable& to);
};

std::ostream& operator<<(std::ostream& stream, const std::shared_ptr<ir_instruction>& item);


#endif //BADJVM_IR_COMPILER_H
