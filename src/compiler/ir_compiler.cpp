#include "ir_compiler.h"
#include "../utils/utils.h"
#include "code_builder.h"
#include "register_shortcuts.h"
#include "code_manager.h"
#include <unordered_set>
#include <queue>

using namespace std;

ir_compiler::ir_compiler(code_manager& manager)
        : manager(manager), reg_list{rax, rcx, rdx, r8, r9, r10, r11} {}

ir_basic_block::ir_basic_block(const ir_label& label) : label(label) {}

int ir_compiler::ir_offset() const {
    return ir.size();
}

ir_label ir_compiler::create_label() {
    return ir_label(++last_label_id);
}

void ir_compiler::add_label(ir_label label, int ir_offset) {
    offset_to_label.insert_or_assign(ir_offset, label);
}

void ir_compiler::convert_to_ssa() {
    unordered_map<ir_label, vector<ir_label>> control_flow_in;
    ir_label root(0);
    blocks.push_back(ir_basic_block(root));

    ir_label current_label = root;
    for (int ir_offset = 0; ir_offset < ir.size(); ir_offset++) {
        auto label_iter = offset_to_label.find(ir_offset);
        if (label_iter != offset_to_label.end()) {
            auto& block = blocks.back();
            if (block.ir.back()->get_jump_labels().empty() && !block.ir.back()->exit_function()) {
                block.ir.push_back(make_shared<ir_jump_instruction>(label_iter->second));
            }
            current_label = label_iter->second;
            blocks.push_back(ir_basic_block(current_label));
        }

        auto& block = blocks.back();
        assert(block.label == current_label)
        block.ir.push_back(ir[ir_offset]);
    }

    unordered_set<int> all_var_ids;
    for (const auto& block : blocks) {
        for (int i = 0; i < block.ir.size(); i++) {
            for (const auto& value : block.ir[i]->get_in_values()) {
                if (value->mode != ir_value_mode::var) continue;
                all_var_ids.insert(value->var.id);
            }
            for (const auto& var : block.ir[i]->get_out_variables()) {
                all_var_ids.insert(var->id);
            }

            const auto& jump_labels = block.ir[i]->get_jump_labels();
            if (i < block.ir.size() - 1) {
                assert(jump_labels.empty())
                continue;
            }
            assert(!jump_labels.empty() || block.ir[i]->exit_function())
            for (const auto& target_label : jump_labels) {
                control_flow_in[target_label].push_back(block.label);
            }
        }
    }

    unordered_map<int, int> last_var_version;
    for (auto& block : blocks) {
        for (const auto& var_id : all_var_ids) {
            vector<pair<ir_label, ir_value>> edges;
            for (const auto& from_label : control_flow_in[block.label]) {
                edges.emplace_back(from_label, ir_variable(var_id, 0));
            }
            block.ir.insert(block.ir.begin(), make_shared<ir_phi_instruction>(edges, ir_variable(var_id, 0)));
        }
    }

    for (auto& block : blocks) {
        for (const auto& instruction : block.ir) {
            for (auto value : instruction->get_in_values()) {
                if (value->mode != ir_value_mode::var) continue;
                value->var.version = last_var_version[value->var.id];
            }
            for (const auto& var : instruction->get_out_variables()) {
                var->version = ++last_var_version[var->id];
            }
        }
        for (const auto& label : block.ir.back()->get_jump_labels()) {
            for (auto& target_block : blocks) {
                if (target_block.label != label) continue;
                for (const auto& instruction : target_block.ir) {
                    if (instruction->tag != ir_instruction_tag::phi) continue;
                    auto phi_instruction = static_pointer_cast<ir_phi_instruction>(instruction);
                    for (auto& [from_label, value] : phi_instruction->edges) {
                        if (from_label != block.label || value.mode != ir_value_mode::var) continue;
                        value.var.version = last_var_version[value.var.id];
                    }
                }
                break;
            }
        }
    }
}

bool remove_unused_vars(vector<ir_basic_block>& blocks) {
    bool changed = false;
    unordered_set<ir_variable> used_vars;
    for (const auto& block : blocks) {
        for (const auto& instruction : block.ir) {
            for (const auto& value : instruction->get_in_values()) {
                if (value->mode != ir_value_mode::var) continue;
                used_vars.insert(value->var);
            }
        }
    }
    for (auto& block : blocks) {
        auto new_end = remove_if(block.ir.begin(), block.ir.end(), [&](const shared_ptr<ir_instruction>& instr) {
            if (!instr->is_pure()) return false;
            auto out_vars = instr->get_out_variables();
            if (out_vars.empty()) return false;
            for (const auto var : out_vars) {
                if (used_vars.find(*var) != used_vars.end()) return false;
            }
            return true;
        });
        if (new_end == block.ir.end()) continue;
        changed = true;
        block.ir.erase(new_end, block.ir.end());
    }
    return changed;
}

bool inline_assigns(vector<ir_basic_block>& blocks) {
    bool changed = false;
    unordered_map<ir_variable, ir_value> assign_map;
    for (const auto& block : blocks) {
        for (const auto& item : block.ir) {
            if (item->tag != ir_instruction_tag::assign) continue;
            auto instruction = static_pointer_cast<ir_assign_instruction>(item);
            assign_map.insert_or_assign(instruction->to, instruction->from);
        }
    }
    for (auto& block : blocks) {
        for (auto& instruction : block.ir) {
            for (auto value : instruction->get_in_values()) {
                if (value->mode != ir_value_mode::var) continue;
                auto iter = assign_map.find(value->var);
                if (iter == assign_map.end()) continue;
                *value = iter->second;
                changed = true;
            }
        }
    }
    return changed;
}

bool eliminate_trivial_phi(vector<ir_basic_block>& blocks) {
    bool changed = false;
    for (auto& block : blocks) {
        for (int i = 0; i < block.ir.size(); i++) {
            if (block.ir[i]->tag != ir_instruction_tag::phi) continue;
            auto instruction = static_pointer_cast<ir_phi_instruction>(block.ir[i]);
            if (instruction->edges.size() != 1) continue;
            block.ir[i] = make_shared<ir_assign_instruction>(instruction->edges[0].second, instruction->to);
            changed = true;
        }
    }
    return changed;
}

void ir_compiler::optimize() {
    bool changed = true;
    while (changed) {
        changed = false;
        changed |= remove_unused_vars(blocks);
        changed |= inline_assigns(blocks);
        changed |= eliminate_trivial_phi(blocks);
    }

    cout << endl << "Optimized SSA IR:" << endl;
    for (const auto& block : blocks) {
        pretty_print(cout, block);
    }
}

void ir_compiler::compile_phi_before_jump(const ir_label& current_label, const ir_basic_block* target_block) {
    // TODO check phi swap problem, maybe group all phi's ang treat them together in liveness analysis
    for (const auto& item : target_block->ir) {
        if (item->tag != ir_instruction_tag::phi) break; // assume all phi's are in the beginning of a block
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        for (const auto& [a_label, value] : instruction->edges) {
            if (a_label != current_label) continue;
            compile_assign(reg_list[color[instruction->to]], value);
            break;
        }
    }
}

const uint8_t* ir_compiler::compile_ssa() {
    cout << endl;
    assert(blocks[0].label.id == 0) // must start from root block
    for (int i = 0; i < blocks.size(); i++) {
        const auto& block = blocks[i];
        builder.mark_label(block.label.id);
        for (int j = 0; j < block.ir.size(); j++) {
            const auto& item = block.ir[j];
            switch (item->tag) {
                case ir_instruction_tag::assign: {
                    auto instruction = static_pointer_cast<ir_assign_instruction>(item);
                    auto to = reg_list[color[instruction->to]];
                    auto from_value = instruction->from;
                    compile_assign(to, from_value);
                    break;
                }
                case ir_instruction_tag::bin_op: {
                    auto instruction = static_pointer_cast<ir_bin_op_insruction>(item);
                    auto to = reg_list[color[instruction->to]];
                    switch (instruction->first.mode) {
                        case ir_value_mode::var: {
                            auto first = reg_list[color[instruction->first.var]];
                            switch (instruction->second.mode) {
                                case ir_value_mode::var: {
                                    auto second = reg_list[color[instruction->second.var]];
                                    switch (instruction->op) {
                                        case ir_bin_op::add: {
                                            if (first == to) {
                                                builder.add(second, to);
                                            } else if (second == to) {
                                                builder.add(first, to);
                                            } else {
                                                builder.mov(first, to);
                                                builder.add(second, to);
                                            }
                                            break;
                                        }
                                        case ir_bin_op::sub: {
                                            if (first == to) {
                                                builder.sub(second, to);
                                            } else if (second == to) {
                                                // TODO get a temp register
                                                builder.mov(second, r11);
                                                builder.mov(first, to);
                                                builder.sub(r11, to);
                                            } else {
                                                builder.mov(first, to);
                                                builder.sub(second, to);
                                            }
                                            break;
                                        }
                                        default: assert(false);
                                    }
                                    break;
                                }
                                case ir_value_mode::int64: {
                                    // TODO implement add with int64
                                    auto value = instruction->second.value;
                                    assert(static_cast<int32_t>(value) == value)
                                    if (first != to) {
                                        builder.mov(first, to);
                                    }
                                    builder.add(value, to);
                                    break;
                                }
                                default: assert(false)
                            }
                            break;
                        }
                        case ir_value_mode::int64: {
                            // TODO such instructions should be eliminated by the optimizer
                            if (instruction->second.mode == ir_value_mode::int64 && instruction->op == ir_bin_op::add) {
                                builder.mov(instruction->first.value + instruction->second.value, reg_list[color[instruction->to]]);
                                break;
                            }
                            assert(false)
                        }
                        default: assert(false)
                    }
                    break;
                }
                case ir_instruction_tag::cmp_jump: {
                    auto instruction = static_pointer_cast<ir_cmp_jump_instruction>(item);
                    auto first = instruction->first;
                    auto second = instruction->second;
                    // TODO implement cmp with imm
                    if (first.mode == ir_value_mode::var && second.mode == ir_value_mode::int64) {
                        auto first_reg = reg_list[color[first.var]];
                        auto second_value = second.value;
                        builder.mov(second_value, r11); // TODO create a temp reg
                        builder.cmp(first_reg, r11);
                    } else {
                        assert(false);
                    }

                    switch (instruction->mode) {
                        case ir_cmp_mode::eq: assert(false)
                        case ir_cmp_mode::neq: assert(false)
                        case ir_cmp_mode::lt: assert(false)
                        case ir_cmp_mode::le: assert(false)
                        case ir_cmp_mode::gt: assert(false)
                        case ir_cmp_mode::ge: {
                            builder.jge(instruction->label_true.id);
                            // TODO insert a new block with phi's impl
                            assert(block_map[instruction->label_true]->ir[0]->tag != ir_instruction_tag::phi)
                            compile_phi_before_jump(block.label, block_map[instruction->label_false]);
                            if (i >= blocks.size() - 1 || blocks[i + 1].label != instruction->label_false) {
                                builder.jmp(instruction->label_false.id);
                            }
                            break;
                        }
                    }
                    break;
                }
                case ir_instruction_tag::jump: {
                    assert(j == block.ir.size() - 1)
                    auto instruction = static_pointer_cast<ir_jump_instruction>(item);
                    auto label = instruction->label;
                    compile_phi_before_jump(block.label, block_map[label]);
                    if (i >= blocks.size() - 1 || blocks[i + 1].label != label) {
                        builder.jmp(label.id);
                    }
                    break;
                }
                case ir_instruction_tag::ret: {
                    auto instruction = static_pointer_cast<ir_ret_instruction>(item);
                    switch (instruction->value.mode) {
                        case ir_value_mode::var: {
                            auto from = reg_list[color[instruction->value.var]];
                            if (from != rax) {
                                builder.mov(from, rax);
                            }
                            break;
                        }
                        case ir_value_mode::int64: {
                            builder.mov(instruction->value.value, rax);
                            break;
                        }
                        default: assert(false)
                    }
                    builder.ret();
                    break;
                }
                case ir_instruction_tag::phi: {
                    break;
                }
                default: assert(false)
            }
        }
    }

    builder.resolve_labels();

    auto code_chunk = manager.add_code_chunk(builder.get_code());

    cout << endl;
    for (const auto& byte : builder.get_code()) {
        printf("%02X ", byte);
    }
    cout << endl << endl;

    return code_chunk;
}

void ir_compiler::compile_assign(jit_register64& to, const ir_value& from_value) {
    switch (from_value.mode) {
        case ir_value_mode::var: {
            auto from = reg_list[color[from_value.var]];
            if (from != to) {
                builder.mov(from, to);
            }
            break;
        }
        case ir_value_mode::int64: {
            builder.mov(from_value.value, to);
            break;
        }
        default: assert(false)
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
        for (const auto& succ_label : control_flow_out[block->label]) {
            live.insert(live_vars[succ_label].begin(), live_vars[succ_label].end());
        }
        for (auto iter = block->ir.rbegin(); iter != block->ir.rend(); ++iter) {
            auto instruction = *iter;
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
        for (const auto& succ_label : control_flow_out[block.label]) {
            live.insert(live_vars[succ_label].begin(), live_vars[succ_label].end());
        }
        for (auto iter = block.ir.rbegin(); iter != block.ir.rend(); ++iter) {
            auto instruction = *iter;
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

    // color[var] == 0 => still not colored
    for (const auto& [var, set] : interference_graph) {
        unordered_set<int> used_colors;
        for (const auto& other_var : set) {
            used_colors.insert(color[other_var]);
        }
        for (int i = 1; i < interference_graph.size() + 2; i++) {
            if (used_colors.find(i) != used_colors.end()) continue;
            color[var] = i;
            break;
        }
        assert(color[var] != 0)
    }
}

const uint8_t* ir_compiler::compile() {
    convert_to_ssa();
    optimize();
    color_variables();
    return compile_ssa();
}
