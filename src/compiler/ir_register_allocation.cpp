#include "ir_compiler.h"
#include "register_shortcuts.h"
#include "../utils/utils.h"
#include <queue>

using namespace std;

void ir_compiler::calculate_color_preferences() {
    unordered_map<ir_variable, unordered_map<ir_variable, int>> score;
    for (const auto& block : blocks) {
        for (const auto& item : block.ir) {
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
        for (const auto& item : succ_block->ir) {
            if (item->tag != ir_instruction_tag::phi) break;
            auto phi_instruction = static_pointer_cast<ir_phi_instruction>(item);
            live.erase(phi_instruction->to);
        }
        for (const auto& item : succ_block->ir) {
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
    unordered_map<ir_label, vector<ir_label>> control_flow_in;
    unordered_map<ir_label, vector<ir_label>> control_flow_out;
    unordered_map<ir_label, unordered_set<ir_variable>> live_vars;

    for (const auto& block : blocks) {
        block_map[block.label] = &block;
        const auto& last_instruction = block.ir.back();
        for (const auto& target_label : last_instruction->get_jump_labels()) {
            control_flow_in[target_label].push_back(block.label);
            control_flow_out[block.label].push_back(target_label);
        }
    }

    queue<ir_label> work_list;
    unordered_set<ir_label> in_list;
    for (auto iter = blocks.rbegin(); iter != blocks.rend(); ++iter) {
        work_list.push(iter->label);
        in_list.insert(iter->label);
    }

    while (!work_list.empty()) {
        auto label = work_list.front();
        work_list.pop();
        in_list.erase(label);
        auto block = block_map[label];
        unordered_set<ir_variable> live;
        merge_succ_blocks(live, live_vars, control_flow_out, block_map, block);

        for (auto iter = block->ir.rbegin(); iter != block->ir.rend(); ++iter) {
            auto instruction = *iter;
            if (instruction->tag == ir_instruction_tag::phi) continue;
            for (auto var : instruction->get_out_variables()) {
                live.erase(*var);
            }
            for (const auto& value : instruction->get_in_values()) {
                if (value->mode != ir_value_mode::var) continue;
                live.insert(value->var);
            }
        }
        assert(live.size() != live_vars[label].size() || live == live_vars[label])
        if (live.size() == live_vars[label].size())
            continue;

        live_vars[label] = live;
        for (const auto& pred_label : control_flow_in[label]) {
            if (in_list.find(pred_label) == in_list.end()) {
                work_list.push(pred_label);
                in_list.insert(pred_label);
            }
        }
    }

    unordered_map<ir_variable, unordered_set<ir_variable>> interference_graph;
    for (const auto& block : blocks) {
        unordered_set<ir_variable> live;
        merge_succ_blocks(live, live_vars, control_flow_out, block_map, &block);
        for (auto iter = block.ir.rbegin(); iter != block.ir.rend(); ++iter) {
            auto instruction = *iter;
            if (instruction->tag == ir_instruction_tag::phi) continue;
            for (auto var : instruction->get_out_variables()) {
                live.erase(*var);
            }
            for (const auto& value : instruction->get_in_values()) {
                if (value->mode != ir_value_mode::var) continue;
                live.insert(value->var);
            }

            for (const auto& var1 : live) {
                for (const auto& var2 : live) {
                    if (var1 == var2) continue;
                    interference_graph[var1].insert(var2);
                    interference_graph[var2].insert(var1);
                }
            }
        }
    }

    calculate_color_preferences();
    unordered_map<jit_register64, int> reg_to_color;
    for (int i = 1; i < reg_list.size(); i++) {
        reg_to_color[reg_list[i]] = i;
    }
    cout << endl << "---- Coloring preference ----" << endl;
    for (const auto& [var, pref_set] : reg_preference) {
        unordered_set<int> used_colors;
        for (const auto& other_var : interference_graph[var]) {
            used_colors.insert(color[other_var]);
        }
        for (const auto& target_reg : pref_set) {
            auto preferred_color = reg_to_color[target_reg];
            if (preferred_color == 0) continue;
            if (used_colors.find(preferred_color) != used_colors.end()) continue;
            cout << "Picked preferred register: " << var << " => " << target_reg << endl;
            color[var] = preferred_color;
            break;
        }
        assert(color[var] != 0)
        assert(color[var] < reg_list.size())
        assert(used_colors.find(color[var]) == used_colors.end())
    }

    for (const auto& [var, set] : interference_graph) {
        if (color[var] != 0) continue;

        unordered_set<int> used_colors;
        for (const auto& other_var : set) {
            used_colors.insert(color[other_var]);
        }
        for (const auto& target_var : color_preference[var]) {
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
        assert(color[var] < reg_list.size())
        assert(used_colors.find(color[var]) == used_colors.end())
    }

    cout << endl << "---- Variables coloring ----" << endl;
    for (const auto& [var, _] : interference_graph) {
        cout << var << " -> " << color[var] << endl;
    }
}