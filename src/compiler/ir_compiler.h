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
#include <algorithm>

struct ir_data_flow_holder {
    std::unordered_set<ir_variable> live_vars;
};

struct ir_data_flow_pair {
    const ir_data_flow_holder& data_in;
    const ir_data_flow_holder& data_out;

    ir_data_flow_pair(const ir_data_flow_holder& data_in, const ir_data_flow_holder& data_out)
            : data_in(data_in), data_out(data_out) {}
};

struct ir_basic_block {
    ir_label label;
    bool data_flow_valid;
    ir_data_flow_holder out_data_flow;
    std::vector<std::pair<std::shared_ptr<ir_instruction>, ir_data_flow_holder>> _ir;

    ir_basic_block(const ir_label& label);
    const std::vector<std::pair<std::shared_ptr<ir_instruction>, ir_data_flow_holder>>& ir() const;
    void set(int index, const std::shared_ptr<ir_instruction>& instruction);
    void insert(int index, const std::shared_ptr<ir_instruction>& instruction);
    void insert_all(int index, const std::vector<std::shared_ptr<ir_instruction>>& instructions);
    void push_back(const std::shared_ptr<ir_instruction>& instruction);

    template <class Pred>
    bool erase_if(const Pred& filter) {
        auto new_end = std::remove_if(_ir.begin(), _ir.end(), [&filter](const auto& pair) {
            const auto& [instr, _] = pair;
            return filter(instr);
        });
        if (new_end == _ir.end()) return false;
        _ir.erase(new_end, _ir.end());
        data_flow_valid = false;
        return true;
    }

    template <class T, class... Args>
    void emplace_at(int index, Args... args) {
        _ir[index] = make_pair(std::make_shared<T>(args...), ir_data_flow_holder());
        data_flow_valid = false;
    }

    template <class T, class... Args>
    void emplace(int index, Args... args) {
        _ir.emplace(_ir.begin() + index, std::make_shared<T>(args...), ir_data_flow_holder());
        data_flow_valid = false;
    }

    template <class T, class... Args>
    void emplace_back(Args... args) {
        _ir.emplace_back(std::make_shared<T>(args...), ir_data_flow_holder());
        data_flow_valid = false;
    }
};

class ir_compiler {
    code_manager& manager;
    code_builder builder;
    std::vector<std::shared_ptr<ir_instruction>> non_ssa_ir;
    std::unordered_map<int, int> last_var_version;
    std::unordered_map<int, ir_label> offset_to_label;
    std::unordered_map<ir_label, ir_label> canonical_label_map;
    int last_label_id = 0;
    std::unordered_map<ir_label, std::vector<ir_label>> control_flow_in;
    std::unordered_map<ir_label, std::vector<ir_label>> control_flow_out;
    std::vector<ir_basic_block> blocks;

    // Valid after coloring
    std::unordered_map<ir_variable, std::vector<ir_variable>> color_preference;
    std::unordered_map<ir_variable, std::unordered_set<jit_register64>> reg_preference;
    std::unordered_map<ir_variable, int> color;
    std::unordered_map<ir_label, ir_basic_block*> block_map;
    std::vector<jit_register64> int_reg_list;
    std::vector<jit_register64> float_reg_list;

public:
    ir_compiler(code_manager& manager);
    int ir_offset() const;
    ir_label create_label();
    void add_label(ir_label label, int ir_offset);
    ir_label create_new_block(const ir_label& position, const ir_label& goto_block);
    void pretty_print(std::ostream& stream);
    void pretty_print(std::ostream& stream, const ir_basic_block& block);
    void compile_assign(const ir_value& from_value, const jit_value_location& to);
    void compile_assign(const jit_value_location& from, const jit_value_location& to);
    void compile_phi_dfs(const jit_value_location& start, int version,
                         const std::unordered_map<jit_value_location, std::vector<jit_value_location>>& assign_from_map,
                         std::unordered_map<jit_value_location, int>& visited_version,
                         std::unordered_map<jit_value_location, jit_value_location>& temp,
                         const ir_data_flow_pair& data);
    void compile_phi_before_jump(const ir_label& current_label, const ir_basic_block* target_block);
    void compile_int32_power2_div(jit_value_location first, int32_t second, jit_value_location to, const ir_data_flow_pair& data);
    void compile_int32_div_const(jit_value_location first, int32_t second, jit_value_location to, const ir_data_flow_pair& data);
    void compile_int_rem(jit_value_location first, jit_value_location second, jit_value_location to, const ir_data_flow_pair& data);
    void compile_double_bin_op(const std::shared_ptr<ir_bin_op_insruction>& instruction, const ir_data_flow_pair& data);
    void compile_bin_op(const std::shared_ptr<ir_bin_op_insruction>& instruction, const ir_data_flow_pair& data);
    jit_value_location get_location(const ir_variable& var);
    bool has_actual_phi_assigns(const ir_basic_block& block, const ir_label& from);
    jit_register64 get_temp_register(const ir_data_flow_pair& data, const ir_storage_type& storage,
                                     const jit_register64& preferred = jit_register64::no_register,
                                     const std::vector<jit_register64>& occupied = {});
    const uint8_t* compile_ssa();
    void calculate_color_preferences();
    void color_variables();
    bool inverse_loops();
    void optimize();
    void build_control_flow_graph();
    void convert_to_ssa();
    const uint8_t* compile();

    void load_argument(int argument_index, ir_variable to);
    void assign(ir_value from, ir_variable to);
    void bin_op(ir_value first, ir_value second, ir_variable to, ir_bin_op op, ir_cmp_nan_mode nan_mode = ir_cmp_nan_mode::no_nan);
    void convert(ir_value from, ir_variable to, ir_convert_mode mode);
    void cmp_jump(ir_value first, ir_value second, ir_cmp_mode mode, ir_label label_true, ir_label label_false, ir_cmp_nan_mode nan_mode);
    void jump(ir_label label);
    void ret(ir_value value);
    void phi(const std::vector<std::pair<ir_label, ir_value>>& edges, const ir_variable& to);

    template <class State, class Transfer, class Merge, class Getter, class Setter, class Equal>
    friend void backward_analysis(ir_compiler& compiler, const Transfer& transfer, const Merge& merge, const Getter& getter, const Setter& setter, const Equal& equal);

    template <class State, class Getter, class Merge>
    friend State merge_succ_blocks(ir_compiler& compiler, const ir_basic_block& block, const Merge& merge, const Getter& getter);
};

std::ostream& operator<<(std::ostream& stream, const std::shared_ptr<ir_instruction>& item);


#endif //BADJVM_IR_COMPILER_H
