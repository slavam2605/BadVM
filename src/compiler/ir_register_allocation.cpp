#include "ir_compiler.h"
#include "ir_data_flow.h"
#include "register_shortcuts.h"
#include "../utils/utils.h"
#include <queue>

using namespace std;

void ir_compiler::calculate_color_preferences() {
    unordered_map<ir_variable, unordered_map<ir_variable, int>> score;
    for (const auto& block : blocks) {
        for (const auto& [item, _] : block.ir()) {
            switch (item->tag) {
                case ir_instruction_tag::phi: {
                    auto instruction = static_pointer_cast<ir_phi_instruction>(item);
                    for (const auto&[_, value] : instruction->edges) {
                        if (value.mode != ir_value_mode::var) continue;
                        if (value.var == instruction->to) continue;
                        score[instruction->to][value.var]++;
                        score[value.var][instruction->to]++;
                    }
                    break;
                }
                case ir_instruction_tag::bin_op: {
                    auto instruction = static_pointer_cast<ir_bin_op_insruction>(item);
                    auto first = instruction->first;
                    auto second = instruction->second;
                    auto to = instruction->to;
                    switch (instruction->op) {
                        case ir_bin_op::add:
                        case ir_bin_op::sub:
                        case ir_bin_op::mul: {
                            if (first.mode == ir_value_mode::var) {
                                score[to][first.var]++;
                                score[first.var][to]++;
                            }
                            break;
                        }
                        case ir_bin_op::rem: {
                            if (first.mode == ir_value_mode::var) {
                                reg_preference[first.var].insert(rax);
                            }
                            reg_preference[to].insert(rdx);
                            break;
                        }
                        default: break;
                    }
                }
                default: break;
            }
        }
    }

    for (const auto& [var, map] : score) {
        vector<pair<ir_variable, int>> to_vector;
        for (const auto& item : map) {
            to_vector.push_back(item);
        }
        sort(to_vector.begin(), to_vector.end(), [](const auto& a, const auto& b) {
            return a.second > b.second;
        });
        for (const auto& [to_var, _] : to_vector) {
            color_preference[var].push_back(to_var);
        }
    }
}

void merge_succ_blocks(unordered_set<ir_variable>& live, unordered_map<ir_label, unordered_set<ir_variable>> live_vars,
                       unordered_map<ir_label, vector<ir_label>>& control_flow_out,
                       unordered_map<ir_label, const ir_basic_block*>& block_map, const ir_basic_block* block) {
    for (const auto& succ_label : control_flow_out[block->label]) {
        live.insert(live_vars[succ_label].begin(), live_vars[succ_label].end());
        auto succ_block = block_map[succ_label];
        for (const auto& [item, _] : succ_block->ir()) {
            if (item->tag != ir_instruction_tag::phi) break;
            auto phi_instruction = static_pointer_cast<ir_phi_instruction>(item);
            live.erase(phi_instruction->to);
        }
        for (const auto& [item, _] : succ_block->ir()) {
            if (item->tag != ir_instruction_tag::phi) break;
            auto phi_instruction = static_pointer_cast<ir_phi_instruction>(item);
            for (const auto& [from_label, value] : phi_instruction->edges) {
                if (from_label != block->label) continue;
                if (value.mode != ir_value_mode::var) break;
                live.insert(value.var);
                break;
            }
        }
    }
}

void ir_compiler::color_variables() {
    backward_analysis<unordered_set<ir_variable>>(
            *this,
            [](auto& state, const auto& instruction) {
                if (instruction->tag == ir_instruction_tag::phi) return;
                for (auto var : instruction->get_out_variables()) {
                    state.erase(*var);
                }
                for (const auto& value : instruction->get_in_values()) {
                    if (value->mode != ir_value_mode::var) continue;
                    state.insert(value->var);
                }
            },
            [](auto& state, const auto& current_block, const auto& succ_block, const auto& in_state) {
                state.insert(in_state.begin(), in_state.end());
                for (const auto& [item, _] : succ_block.ir()) {
                    if (item->tag != ir_instruction_tag::phi) break;
                    auto phi_instruction = static_pointer_cast<ir_phi_instruction>(item);
                    state.erase(phi_instruction->to);
                }
                for (const auto& [item, _] : succ_block.ir()) {
                    if (item->tag != ir_instruction_tag::phi) break;
                    auto phi_instruction = static_pointer_cast<ir_phi_instruction>(item);
                    for (const auto& [from_label, value] : phi_instruction->edges) {
                        if (from_label != current_block.label) continue;
                        if (value.mode != ir_value_mode::var) break;
                        state.insert(value.var);
                        break;
                    }
                }
            },
            [](const auto& data_flow_holder) { return data_flow_holder.live_vars; },
            [](auto& data_flow_holder, const auto& state) { data_flow_holder.live_vars = state; },
            [](const auto& holder1, const auto& holder2) { return holder1.live_vars.size() == holder2.live_vars.size(); }
    );

    unordered_map<ir_variable, unordered_set<ir_variable>> interference_graph;
    for (const auto& block : blocks) {
        for (auto iter = block.ir().rbegin(); iter != block.ir().rend(); ++iter) {
            const auto& [_, holder] = *iter;
            for (const auto& var1 : holder.live_vars) {
                for (const auto& var2 : holder.live_vars) {
                    if (var1 == var2) continue;
                    interference_graph[var1].insert(var2);
                    interference_graph[var2].insert(var1);
                }
            }
        }
    }

    calculate_color_preferences();
    unordered_map<jit_register64, int> int_reg_to_color;
    unordered_map<jit_register64, int> float_reg_to_color;
    for (int i = 1; i < int_reg_list.size(); i++) {
        int_reg_to_color[int_reg_list[i]] = i;
    }
    for (int i = 1; i < float_reg_list.size(); i++) {
        float_reg_to_color[float_reg_list[i]] = i;
    }

    cout << endl << "---- Coloring preference ----" << endl;

    // Assign locations for function arguments
    vector<jit_register64> int_arg_regs {rcx, rdx, r8, r9};
    vector<jit_register64> float_arg_regs {xmm0, xmm1, xmm2, xmm3};
    for (const auto& block : blocks) {
        if (block.label.id != 0) continue;
        for (const auto& [item, _] : block.ir()) {
            if (item->tag != ir_instruction_tag::load_argument) continue;
            auto instruction = static_pointer_cast<ir_load_argument_instruction>(item);
            auto storage = get_storage_type(instruction->to.type);
            switch (storage) {
                case ir_storage_type::ir_int: {
                    auto argument_reg = int_arg_regs[instruction->argument_index];
                    cout << "Set register for the argument " << instruction->argument_index << ": " << instruction->to << " => " << argument_reg << endl;
                    color[instruction->to] = int_reg_to_color[argument_reg];
                    break;
                }
                case ir_storage_type::ir_float: {
                    auto argument_reg = float_arg_regs[instruction->argument_index];
                    cout << "Set register for the argument " << instruction->argument_index << ": " << instruction->to << " => " << argument_reg << endl;
                    color[instruction->to] = float_reg_to_color[argument_reg];
                    break;
                }
                default_fail
            }
        }
    }

    unordered_set<ir_variable> all_variables;
    for (const auto& block : blocks) {
        for (const auto& [item, _] : block.ir()) {
            for (const auto& var : item->get_out_variables()) {
                all_variables.insert(*var);
            }
        }
    }

    for (const auto& [var, pref_set] : reg_preference) {
        auto storage = get_storage_type(var.type);
        if (storage != ir_storage_type::ir_int) assert(false)
        unordered_set<int> used_colors;
        for (const auto& other_var : interference_graph[var]) {
            if (get_storage_type(other_var.type) != storage) continue;
            used_colors.insert(color[other_var]);
        }
        for (const auto& target_reg : pref_set) {
            auto preferred_color = int_reg_to_color[target_reg];
            if (preferred_color == 0) continue;
            if (used_colors.find(preferred_color) != used_colors.end()) continue;
            cout << "Picked preferred register: " << var << " => " << target_reg << endl;
            color[var] = preferred_color;
            break;
        }
        assert(color[var] != 0)
        assert(storage != ir_storage_type::ir_int || color[var] < int_reg_list.size())
        assert(used_colors.find(color[var]) == used_colors.end())
    }

    for (const auto& var : all_variables) {
        if (color[var] != 0) continue;
        auto storage = get_storage_type(var.type);

        unordered_set<int> used_colors;
        for (const auto& other_var : interference_graph[var]) {
            if (get_storage_type(other_var.type) != storage) continue;
            used_colors.insert(color[other_var]);
        }
        for (const auto& target_var : color_preference[var]) {
            assert(get_storage_type(target_var.type) == storage)
            auto preferred_color = color[target_var];
            if (preferred_color == 0) continue;
            if (used_colors.find(preferred_color) != used_colors.end()) continue;
            cout << "Picked preferred color: " << var << " <=> " << target_var << endl;
            color[var] = preferred_color;
            goto color_done;
        }
        for (int i = 1; i <= interference_graph.size() + 1; i++) {
            if (used_colors.find(i) != used_colors.end()) continue;
            color[var] = i;
            break;
        }

        color_done:
        assert(color[var] != 0)
        assert(storage != ir_storage_type::ir_int || color[var] < int_reg_list.size())
        assert(storage != ir_storage_type::ir_float || color[var] < float_reg_list.size())
        assert(used_colors.find(color[var]) == used_colors.end())
    }

    cout << endl << "---- Variables coloring ----" << endl;
    for (const auto& [var, _] : interference_graph) {
        cout << var << " -> " << color[var] << endl;
    }
}