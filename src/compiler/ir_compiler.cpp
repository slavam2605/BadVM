#include "ir_compiler.h"
#include "../utils/utils.h"
#include "register_shortcuts.h"
#include <unordered_set>

#define assert_fit_int32(x) assert(static_cast<int32_t>(x) == x)

using namespace std;

namespace std {
    template<>
    struct hash<jit_value_location> {
        size_t operator()(const jit_value_location& location) const {
            return hash<jit_register64>()(location.reg) * 1000000 +
                    location.stack_offset * 1000 + location.bit_size;
        }
    };
}

ir_compiler::ir_compiler(code_manager& manager)
        : manager(manager), int_reg_list{no_register, rax, rcx, rdx, r8, r9/*, r10, r11*/},
          float_reg_list{no_register, xmm0, xmm1, xmm2, xmm3, xmm4, xmm5} {}

ir_basic_block::ir_basic_block(const ir_label& label) : label(label) {
    data_flow_valid = false;
}

const vector<pair<shared_ptr<ir_instruction>, ir_data_flow_holder>>& ir_basic_block::ir() const {
    return _ir;
}

void ir_basic_block::set(int index, const std::shared_ptr<ir_instruction>& instruction) {
    _ir[index] = make_pair(instruction, ir_data_flow_holder());
    data_flow_valid = false;
}

void ir_basic_block::insert(int index, const std::shared_ptr<ir_instruction>& instruction) {
    _ir.emplace(_ir.begin() + index, instruction, ir_data_flow_holder());
    data_flow_valid = false;
}

void ir_basic_block::insert_all(int index, const vector<shared_ptr<ir_instruction>>& instructions) {
    for (auto iter = instructions.cbegin(); iter != instructions.cend(); ++iter) {
        _ir.emplace(_ir.begin() + index, *iter, ir_data_flow_holder());
    }
    data_flow_valid = false;
}

void ir_basic_block::push_back(const std::shared_ptr<ir_instruction>& instruction) {
    _ir.emplace_back(instruction, ir_data_flow_holder());
    data_flow_valid = false;
}

int ir_compiler::ir_offset() const {
    return non_ssa_ir.size();
}

ir_label ir_compiler::create_label() {
    return ir_label(++last_label_id);
}

void ir_compiler::add_label(ir_label label, int ir_offset) {
    offset_to_label.insert_or_assign(ir_offset, label);
}

ir_storage_type get_storage_type(const jit_value_location& loc) {
    switch (loc.reg) {
        case jit_register64::rax:
        case jit_register64::rbx:
        case jit_register64::rcx:
        case jit_register64::rdx:
        case jit_register64::rsp:
        case jit_register64::rbp:
        case jit_register64::rsi:
        case jit_register64::rdi:
        case jit_register64::r8:
        case jit_register64::r9:
        case jit_register64::r10:
        case jit_register64::r11:
        case jit_register64::r12:
        case jit_register64::r13:
        case jit_register64::r14:
        case jit_register64::r15:
            return ir_storage_type::ir_int;

        case jit_register64::xmm0:
        case jit_register64::xmm1:
        case jit_register64::xmm2:
        case jit_register64::xmm3:
        case jit_register64::xmm4:
        case jit_register64::xmm5:
        case jit_register64::xmm6:
        case jit_register64::xmm7:
        case jit_register64::xmm8:
        case jit_register64::xmm9:
        case jit_register64::xmm10:
        case jit_register64::xmm11:
        case jit_register64::xmm12:
        case jit_register64::xmm13:
        case jit_register64::xmm14:
        case jit_register64::xmm15:
            return ir_storage_type::ir_float;

        default_fail
    }
}

jit_value_location ir_compiler::get_location(const ir_variable& var) {
    switch (get_storage_type(var.type)) {
        case ir_storage_type::ir_int: {
            auto reg = int_reg_list[color[var]];
            switch (var.type) {
                case ir_variable_type::ir_int: return jit_value_location(reg, 32);
                case ir_variable_type::ir_long: return jit_value_location(reg);
                default_fail
            }
        }
        case ir_storage_type::ir_float: {
            auto reg = float_reg_list[color[var]];
            switch (var.type) {
                case ir_variable_type::ir_double: return jit_value_location(reg, 64);
                default_fail
            }
        }
        default_fail
    }
}

void ir_compiler::build_control_flow_graph() {
    assert(block_map.empty())
    assert(control_flow_in.empty())
    assert(control_flow_out.empty())
    for (auto& block : blocks) {
        block_map[block.label] = &block;
        const auto& [last_instruction, _] = block.ir().back();
        for (const auto& target_label : last_instruction->get_jump_labels()) {
            control_flow_in[*target_label].push_back(block.label);
            control_flow_out[block.label].push_back(*target_label);
        }
    }
}

void ir_compiler::convert_to_ssa() {
    unordered_map<ir_label, vector<ir_label>> control_flow;
    ir_label root(0);
    blocks.push_back(ir_basic_block(root));

    ir_label current_label = root;
    for (int ir_offset = 0; ir_offset < non_ssa_ir.size(); ir_offset++) {
        auto label_iter = offset_to_label.find(ir_offset);
        if (label_iter != offset_to_label.end()) {
            auto& block = blocks.back();
            if (block.ir().back().first->get_jump_labels().empty() && !block.ir().back().first->exit_function()) {
                block.emplace_back<ir_jump_instruction>(label_iter->second);
            }
            current_label = label_iter->second;
            blocks.push_back(ir_basic_block(current_label));
        }

        auto& block = blocks.back();
        assert(block.label == current_label)
        block.push_back(non_ssa_ir[ir_offset]);
    }

    unordered_map<int, ir_variable_type> all_var_ids;
    for (const auto& block : blocks) {
        for (int i = 0; i < block.ir().size(); i++) {
            const auto& [item, _] = block.ir()[i];
            for (const auto& value : item->get_in_values()) {
                if (value->mode != ir_value_mode::var) continue;
                all_var_ids[value->var.id] = value->var.type;
            }
            for (const auto& var : item->get_out_variables()) {
                all_var_ids[var->id] = var->type;
            }

            const auto& jump_labels = item->get_jump_labels();
            if (i < block.ir().size() - 1) {
                assert(jump_labels.empty())
                continue;
            }
            assert(!jump_labels.empty() || item->exit_function())
            for (const auto& target_label : jump_labels) {
                control_flow[*target_label].push_back(block.label);
            }
        }
    }

    for (auto& block : blocks) {
        for (const auto& [var_id, type] : all_var_ids) {
            vector<pair<ir_label, ir_value>> edges;
            for (const auto& from_label : control_flow[block.label]) {
                edges.emplace_back(from_label, ir_variable(var_id, 0, type));
            }
            block.emplace<ir_phi_instruction>(0, edges, ir_variable(var_id, 0, type));
        }
    }

    for (auto& block : blocks) {
        for (const auto& [instruction, _] : block.ir()) {
            if (instruction->tag != ir_instruction_tag::phi) {
                for (auto value : instruction->get_in_values()) {
                    if (value->mode != ir_value_mode::var) continue;
                    value->var.version = last_var_version[value->var.id];
                }
            }
            for (const auto& var : instruction->get_out_variables()) {
                var->version = ++last_var_version[var->id];
            }
        }
        for (const auto& label : block.ir().back().first->get_jump_labels()) {
            for (auto& target_block : blocks) {
                if (target_block.label != *label) continue;
                for (const auto& [instruction, _] : target_block.ir()) {
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

bool find_used_dfs(const unordered_set<ir_variable>& useful, const unordered_set<ir_variable>& unused,
                   const unordered_map<ir_variable, unordered_set<ir_variable>>& affected,
                   unordered_set<ir_variable>& visited, const ir_variable& var) {
    if (visited.find(var) != visited.end()) return false;
    if (unused.find(var) != unused.end()) return false;
    if (useful.find(var) != useful.end()) return true;
    visited.insert(var);
    auto iter = affected.find(var);
    if (iter == affected.end()) return false;
    for (const auto& affected_var : iter->second) {
        if (find_used_dfs(useful, unused, affected, visited, affected_var))
            return true;
    }
    return false;
}

bool remove_unused_vars(vector<ir_basic_block>& blocks) {
    bool changed = false;
    unordered_map<ir_variable, unordered_set<ir_variable>> affected;
    unordered_set<ir_variable> useful;
    unordered_set<ir_variable> assigned;
    for (const auto& block : blocks) {
        for (const auto& [instruction, _] : block.ir()) {
            auto out_vars = instruction->get_out_variables();
            auto useful_instruction = !instruction->is_pure() || out_vars.empty();
            for (const auto& var : out_vars) {
                assigned.insert(*var);
            }
            for (const auto& value : instruction->get_in_values()) {
                if (value->mode != ir_value_mode::var) continue;
                if (useful_instruction) {
                    useful.insert(value->var);
                    continue;
                }
                for (auto out_var : out_vars) {
                    affected[value->var].insert(*out_var);
                }
            }
        }
    }

    unordered_set<ir_variable> visited;
    unordered_set<ir_variable> unused;
    for (const auto& var : assigned) {
        visited.clear();
        if (!find_used_dfs(useful, unused, affected, visited, var))
            unused.insert(var);
    }

    for (auto& block : blocks) {
        auto erased = block.erase_if([&](const auto& instr) {
            if (!instr->is_pure()) return false;
            auto out_vars = instr->get_out_variables();
            if (out_vars.empty()) return false;
            for (const auto var : out_vars) {
                if (unused.find(*var) == unused.end())
                    return false;
            }
            return true;
        });
        changed |= erased;
    }
    return changed;
}

bool inline_assigns(vector<ir_basic_block>& blocks) {
    bool changed = false;
    unordered_map<ir_variable, ir_value> assign_map;
    for (const auto& block : blocks) {
        for (const auto& [item, _] : block.ir()) {
            if (item->tag != ir_instruction_tag::assign) continue;
            auto instruction = static_pointer_cast<ir_assign_instruction>(item);
            assign_map.insert_or_assign(instruction->to, instruction->from);
        }
    }
    for (auto& block : blocks) {
        for (auto& [instruction, _] : block.ir()) {
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
        for (int i = 0; i < block.ir().size(); i++) {
            if (block.ir()[i].first->tag != ir_instruction_tag::phi) continue;
            auto instruction = static_pointer_cast<ir_phi_instruction>(block.ir()[i].first);
            if (instruction->edges.size() == 1) {
                block.emplace_at<ir_assign_instruction>(i, instruction->edges[0].second, instruction->to);
                changed = true;
            } else if (instruction->edges.size() > 1) {
                optional<ir_value> unique;
                for (const auto& [_, value] : instruction->edges) {
                    if (value.mode == ir_value_mode::var && instruction->to == value.var) continue;
                    if (unique.has_value()) goto skip;
                    unique = value;
                }
                assert(unique.has_value())
                block.emplace_at<ir_assign_instruction>(i, *unique, instruction->to);
                changed = true;
                skip:;
            }
        }
    }
    return changed;
}

bool simplify_instructions(vector<ir_basic_block>& blocks) {
    bool changed = false;
    unordered_map<ir_label, unordered_set<ir_label>> reversed_control_flow;
    for (const auto& block : blocks) {
        for (const auto label : block.ir().back().first->get_jump_labels()) {
            reversed_control_flow[*label].insert(block.label);
        }
    }

    for (auto& block : blocks) {
        for (int i = 0; i < block.ir().size(); i++) {
            const auto& [item, _] = block.ir()[i];
            switch (item->tag) {
                case ir_instruction_tag::phi: {
                    auto instruction = static_pointer_cast<ir_phi_instruction>(item);
                    vector<pair<ir_label, ir_value>> new_edges;
                    for (const auto& [label, value] : instruction->edges) {
                        if (reversed_control_flow[block.label].find(label) == reversed_control_flow[block.label].end()) continue;
                        new_edges.emplace_back(label, value);
                    }
                    if (new_edges.size() == instruction->edges.size()) break;
                    instruction->edges = new_edges;
                    changed = true;
                    break;
                }
                default: break;
            }

            for (const auto& item : item->get_in_values()) {
                if (item->mode != ir_value_mode::int64) goto ir_loop_end;
            }
            switch (item->tag) {
                case ir_instruction_tag::bin_op: {
                    auto instruction = static_pointer_cast<ir_bin_op_insruction>(item);
                    auto first = instruction->first.int64_value;
                    auto second = instruction->second.int64_value;
                    int64_t value;
                    switch (instruction->op) {
                        case ir_bin_op::add: value = first + second; break;
                        case ir_bin_op::sub: value = first - second; break;
                        case ir_bin_op::mul: value = first * second; break;
                        case ir_bin_op::div: value = first / second; break;
                        case ir_bin_op::rem: value = first % second; break;
                        default_fail
                    }
                    block.emplace_at<ir_assign_instruction>(i, ir_value(value), instruction->to);
                    changed = true;
                    break;
                }
                case ir_instruction_tag::cmp_jump: {
                    auto instruction = static_pointer_cast<ir_cmp_jump_instruction>(item);
                    auto first = instruction->first.int64_value;
                    auto second = instruction->second.int64_value;
                    bool value;
                    switch (instruction->mode) {
                        case ir_cmp_mode::eq: value = first == second; break;
                        case ir_cmp_mode::neq: value = first != second; break;
                        case ir_cmp_mode::lt: value = first < second; break;
                        case ir_cmp_mode::le: value = first <= second; break;
                        case ir_cmp_mode::gt: value = first > second; break;
                        case ir_cmp_mode::ge: value = first >= second; break;
                        default_fail
                    }
                    auto label = value ? instruction->label_true : instruction->label_false;
                    block.emplace_at<ir_jump_instruction>(i, label);
                    changed = true;
                    break;
                }
                case ir_instruction_tag::load_argument:
                case ir_instruction_tag::assign:
                case ir_instruction_tag::jump:
                case ir_instruction_tag::ret:
                case ir_instruction_tag::phi: break;
                default_fail
            }
            ir_loop_end:;
        }
    }
    return changed;
}

bool remove_unused_blocks(vector<ir_basic_block>& blocks) {
    bool changed = false;

    unordered_map<ir_label, const ir_basic_block*> block_map;
    for (const auto& block : blocks) {
        block_map[block.label] = &block;
    }

    unordered_map<ir_label, ir_label> label_replace;
    unordered_map<ir_label, vector<ir_label>> jump_source;
    for (const auto& block : blocks) {
        if (block.label.id == 0) continue;
        if (block.ir().size() != 1) continue;
        const auto& [item, _] = block.ir()[0];
        if (item->tag != ir_instruction_tag::jump) continue;
        auto instruction = static_pointer_cast<ir_jump_instruction>(item);
        if (block_map[instruction->label]->ir()[0].first->tag == ir_instruction_tag::phi) continue;
        label_replace[block.label] = instruction->label;
        changed = true;
    }
    for (auto& block : blocks) {
        for (const auto& [item, _] : block.ir()) {
            for (auto label : item->get_jump_labels()) {
                auto iter = label_replace.find(*label);
                if (iter == label_replace.end()) continue;
                jump_source[*label].push_back(block.label);
                *label = iter->second;
            }
        }
    }
    for (auto& block : blocks) {
        for (const auto& [item, _] : block.ir()) {
            if (item->tag == ir_instruction_tag::phi) {
                auto instruction = static_pointer_cast<ir_phi_instruction>(item);
                unordered_map<ir_label, ir_value> old_values;
                for (auto& [label, value] : instruction->edges) {
                    auto iter = label_replace.find(label);
                    if (iter == label_replace.end()) continue;
                    old_values.insert_or_assign(label, value);
                }
                auto new_end = remove_if(instruction->edges.begin(), instruction->edges.end(), [&](const auto& pair) {
                    return label_replace.find(pair.first) != label_replace.end();
                });
                instruction->edges.erase(new_end, instruction->edges.end());
                for (const auto& [label, value] : old_values) {
                    for (const auto& new_source : jump_source[label]) {
                        instruction->edges.emplace_back(new_source, value);
                    }
                }
            }
        }
    }

    unordered_set<ir_label> used_labels;
    used_labels.insert(ir_label(0)); // root block
    for (const auto& block : blocks) {
        for (const auto& label : block.ir().back().first->get_jump_labels()) {
            used_labels.insert(*label);
        }
    }
    auto new_end = remove_if(blocks.begin(), blocks.end(), [&](const auto& block){
        return used_labels.find(block.label) == used_labels.end();
    });
    if (new_end != blocks.end()) changed = true;
    blocks.erase(new_end, blocks.end());

    return changed;
}

bool combine_compare_jump(vector<ir_basic_block>& blocks) {
    bool changed = false;
    unordered_map<ir_variable, tuple<ir_value, ir_value, ir_cmp_nan_mode>> compare_assigns;
    for (const auto& block : blocks) {
        for (const auto& [item, _] : block.ir()) {
            if (item->tag != ir_instruction_tag::bin_op) continue;
            auto instruction = static_pointer_cast<ir_bin_op_insruction>(item);
            if (instruction->op != ir_bin_op::cmp) continue;
            compare_assigns.insert_or_assign(instruction->to, make_tuple(instruction->first, instruction->second, instruction->nan_mode));
        }
    }

    for (auto& block : blocks) {
        for (int i = 0; i < block.ir().size(); i++) {
            const auto& [item, _] = block.ir()[i];
            if (item->tag != ir_instruction_tag::cmp_jump) continue;
            auto instruction = static_pointer_cast<ir_cmp_jump_instruction>(item);
            if (instruction->first.mode != ir_value_mode::var) continue;
            auto iter = compare_assigns.find(instruction->first.var);
            if (iter == compare_assigns.end()) continue;
            auto [new_first, new_second, nan_mode] = iter->second;
            block.emplace_at<ir_cmp_jump_instruction>(i, new_first, new_second, instruction->mode,
                                                      instruction->label_true, instruction->label_false, nan_mode);
            changed = true;
        }
    }
    return changed;
}

optional<ir_label> find_while_end(unordered_map<ir_label, ir_basic_block*>& block_map, unordered_set<ir_label>& visited,
                                  const ir_label& label, const ir_label& while_start) {
    if (visited.find(label) != visited.end()) return nullopt;
    visited.insert(label);
    auto block = block_map[label];
    auto& [last, _] = block->ir().back();
    if (last->tag == ir_instruction_tag::jump && static_pointer_cast<ir_jump_instruction>(last)->label == while_start) {
        return label;
    }
    for (const auto& jump_label : last->get_jump_labels()) {
        auto result = find_while_end(block_map, visited, *jump_label, while_start);
        if (result.has_value()) return result;
    }
    visited.erase(label);
    return nullopt;
}

optional<tuple<ir_label, ir_label, ir_label>> find_while_loop(unordered_map<ir_label, ir_basic_block*>& block_map,
                                                              unordered_set<ir_label>& visited, const ir_label& label) {
    if (visited.find(label) != visited.end()) return nullopt;
    visited.insert(label);
    auto block = block_map[label];
    auto& [last, _] = block->ir().back();
    for (const auto& jump_label : last->get_jump_labels()) {
        if (last->tag == ir_instruction_tag::cmp_jump) {
            auto result = find_while_end(block_map, visited, *jump_label, label);
            if (result.has_value()) return make_tuple(label, *jump_label, *result);
        }
        auto result = find_while_loop(block_map, visited, *jump_label);
        if (result.has_value()) return result;
    }
    return nullopt;
}

void rename_variables(unordered_map<ir_label, ir_basic_block*>& block_map, unordered_set<ir_label>& visited,
                      unordered_map<ir_variable, ir_variable>& rename_map, const ir_label& label, const ir_label& ignore_label) {
    if (visited.find(label) != visited.end()) return;
    visited.insert(label);
    auto block = block_map[label];
    for (auto& [item, _] : block->ir()) {
        for (auto value : item->get_in_values()) {
            if (value->mode != ir_value_mode::var) continue;
            auto iter = rename_map.find(value->var);
            if (iter == rename_map.end()) continue;
            value->var = iter->second;
        }
    }
    auto& last = block->ir().back();
    for (const auto& jump_label : last.first->get_jump_labels()) {
        if (*jump_label == ignore_label) continue;
        rename_variables(block_map, visited, rename_map, *jump_label, ignore_label);
    }
}

bool ir_compiler::inverse_loops() {
    unordered_map<ir_label, ir_basic_block*> temp_block_map;
    for (auto& block : blocks) {
        temp_block_map[block.label] = &block;
    }
    unordered_set<ir_label> visited;
    auto result = find_while_loop(temp_block_map, visited, ir_label(0));
    if (!result.has_value()) return false;
    auto [while_start, loop_first, while_end] = *result;
    // TODO check that there are no jumps to loop_first..while_end range
    //  except for while_start -> loop_first and all jumps from this range to itself (excluding loop_first)
    for (auto& block : blocks) {
        if (block.label == while_start) continue;
        for (const auto& jump_label : block.ir().back().first->get_jump_labels()) {
            if (*jump_label == loop_first) return false;
        }
    }

    auto while_start_block = temp_block_map[while_start];
    auto loop_first_block = temp_block_map[loop_first];
    auto while_end_block = temp_block_map[while_end];
    for (auto iter = while_start_block->ir().rbegin(); iter != while_start_block->ir().rend(); ++iter) {
        if (!iter->first->get_jump_labels().empty()) continue;
        loop_first_block->insert(0, iter->first->clone());
    }
    unordered_map<ir_variable, ir_variable> rename_map;
    for (int i = 0; i < while_start_block->ir().size(); i++) {
        const auto& [item, _] = while_start_block->ir()[i];
        if (item->tag != ir_instruction_tag::phi) continue;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        vector<pair<ir_label, ir_value>> new_edges;
        for (const auto& [label, value] : instruction->edges) {
            if (label == while_end) continue;
            new_edges.emplace_back(label, value);
        }
        auto new_version = ++last_var_version[instruction->to.id];
        auto new_var = ir_variable(instruction->to.id, new_version, instruction->to.type);
        rename_map[instruction->to] = new_var;
        while_start_block->emplace_at<ir_phi_instruction>(i, new_edges, new_var);
    }
    for (int i = 0; i < loop_first_block->ir().size(); i++) {
        const auto& [item, _] = loop_first_block->ir()[i];
        if (item->tag != ir_instruction_tag::phi) continue;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        vector<pair<ir_label, ir_value>> new_edges;
        for (const auto& [label, value] : instruction->edges) {
            if (label != while_end) continue;
            new_edges.emplace_back(label, value);
            break;
        }
        new_edges.emplace_back(while_start, ir_variable(instruction->to.id, last_var_version[instruction->to.id], instruction->to.type));
        loop_first_block->emplace_at<ir_phi_instruction>(i, new_edges, instruction->to);
    }
    {
        assert(while_end_block->ir().back().first->tag == ir_instruction_tag::jump)
        auto last_instruction = while_start_block->ir().back().first->clone();
        auto last_jump = static_pointer_cast<ir_cmp_jump_instruction>(last_instruction);
        last_jump->mode = negate_cmp_mode(last_jump->mode);
        swap(last_jump->label_true, last_jump->label_false);
        while_end_block->set(while_end_block->ir().size() - 1, last_jump);
    }
    unordered_map<ir_variable, ir_value> while_end_last_var_map;
    for (const auto& [item, _] : loop_first_block->ir()) {
        if (item->tag != ir_instruction_tag::phi) continue;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        for (const auto& [label, value] : instruction->edges) {
            if (label != while_end) continue;
            while_end_last_var_map.insert_or_assign(instruction->to, value);
        }
    }
    auto [new_jump, _] = while_end_block->ir().back();
    for (auto value : new_jump->get_in_values()) {
        if (value->mode != ir_value_mode::var) continue;
        auto iter = while_end_last_var_map.find(value->var);
        if (iter == while_end_last_var_map.end()) continue;
        *value = iter->second;
    }

    visited.clear();
    rename_variables(temp_block_map, visited, rename_map, while_start, loop_first);
    ir_label after_label;
    for (const auto& jump_label : while_start_block->ir().back().first->get_jump_labels()) {
        if (*jump_label == loop_first) continue;
        after_label = *jump_label;
        break;
    }
    assert(after_label.id >= 0)
    auto after_block = temp_block_map[after_label];

    unordered_map<ir_variable, ir_value> reassign_map;
    for (const auto& [item, _] : loop_first_block->ir()) {
        if (item->tag != ir_instruction_tag::phi) continue;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        optional<ir_variable> from_while_start;
        for (const auto& [label, value] : instruction->edges) {
            if (label != while_start) continue;
            assert(value.mode == ir_value_mode::var)
            from_while_start = value.var;
            break;
        }
        if (!from_while_start.has_value()) continue;
        optional<ir_value> from_while_end;
        for (const auto& [label, value] : instruction->edges) {
            if (label != while_end) continue;
            from_while_end = value;
            break;
        }
        assert(from_while_end.has_value())
        reassign_map.insert_or_assign(*from_while_start, *from_while_end);
    }

    unordered_map<ir_variable, ir_variable> new_rename_map;
    vector<shared_ptr<ir_instruction>> to_add;
    for (const auto& [from_start, from_end] : reassign_map) {
        auto new_version = ++last_var_version[from_start.id];
        auto new_var = ir_variable(from_start.id, new_version, from_start.type);
        new_rename_map[from_start] = new_var;
        vector<pair<ir_label, ir_value>> new_edges{make_pair(while_start, from_start), make_pair(while_end, from_end)};
        auto new_phi = make_shared<ir_phi_instruction>(new_edges, new_var);
        to_add.push_back(new_phi);
    }
    visited.clear();
    rename_variables(temp_block_map, visited, new_rename_map, after_label, ir_label());
    after_block->insert_all(0, to_add);

    return true;
}

void ir_compiler::optimize() {
    bool changed = true;
    while (changed) {
        changed = false;
        changed |= simplify_instructions(blocks);
        changed |= inline_assigns(blocks);
        changed |= combine_compare_jump(blocks);
        changed |= eliminate_trivial_phi(blocks);
        changed |= remove_unused_vars(blocks);
        changed |= inverse_loops();
        changed |= remove_unused_blocks(blocks);
    }

    cout << endl << "---- Optimized SSA IR ----" << endl;
    for (const auto& block : blocks) {
        pretty_print(cout, block);
    }
}

void ir_compiler::compile_phi_dfs(const jit_value_location& start, int version,
                                  const unordered_map<jit_value_location, vector<jit_value_location>>& assign_from_map,
                                  unordered_map<jit_value_location, int>& visited_version,
                                  unordered_map<jit_value_location, jit_value_location>& temp,
                                  const ir_data_flow_pair& data) {
    auto iter = visited_version.find(start);
    if (iter != visited_version.end()) {
        auto found_version = iter->second;
        if (found_version != version) return;
        // loop detected
        auto storage = get_storage_type(start);
        auto temp_reg = get_temp_register(data, storage);
        compile_assign(start, temp_reg);
        temp[start] = temp_reg;
        return;
    }
    visited_version[start] = version;
    auto to_iter = assign_from_map.find(start);
    if (to_iter == assign_from_map.end()) return;
    for (const auto& to : to_iter->second) {
        compile_phi_dfs(to, version, assign_from_map, visited_version, temp, data);
        auto temp_iter = temp.find(start);
        if (temp_iter != temp.end()) {
            compile_assign(temp_iter->second, to);
        } else {
            compile_assign(start, to);
        }
    }
}

void ir_compiler::compile_phi_before_jump(const ir_label& current_label, const ir_basic_block* target_block) {
//    cout << "L" << current_label.id << "_PHI_BLOCK:" << endl;
    unordered_map<jit_value_location, vector<jit_value_location>> assign_from_map;
    vector<jit_value_location> assign_order;
    vector<pair<jit_value_location, ir_value>> not_var_assign_list;

    for (const auto& [item, _] : target_block->ir()) {
        if (item->tag != ir_instruction_tag::phi) break; // assume all phi's are in the beginning of a block
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        for (const auto& [a_label, value] : instruction->edges) {
            if (a_label != current_label) continue;
            if (value.mode == ir_value_mode::var) {
                auto from_loc = get_location(value.var);
                auto to_loc = get_location(instruction->to);
                if (from_loc != to_loc) {
                    if (assign_from_map.find(from_loc) == assign_from_map.end()) {
                        assign_order.push_back(from_loc);
                    }
                    assign_from_map[from_loc].push_back(to_loc);
                }
            } else {
                not_var_assign_list.emplace_back(get_location(instruction->to), value);
            }
            break;
        }
    }

    // TODO add cycle detection and resolution
    int version = 0;
    unordered_map<jit_value_location, int> visited_version;
    unordered_map<jit_value_location, jit_value_location> temp;
    for (const auto& from : assign_order) {
        temp.clear();
        const auto& data_in = target_block->ir()[0].second;
        // TODO set proper data here
        compile_phi_dfs(from, version, assign_from_map, visited_version, temp, ir_data_flow_pair(data_in, data_in));
        version++;
    }
    for (const auto& [to, value] : not_var_assign_list) {
        compile_assign(value, to);
    }
}

bool ir_compiler::has_actual_phi_assigns(const ir_basic_block& block, const ir_label& from) {
    for (const auto& [item, _] : block.ir()) {
        if (item->tag != ir_instruction_tag::phi) break;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        auto to_loc = get_location(instruction->to);
        for (const auto& [label, value] : instruction->edges) {
            if (label != from) continue;
            if (value.mode != ir_value_mode::var) return true;
            auto value_loc = get_location(value.var);
            if (to_loc != value_loc) return true;
            break;
        }
    }
    return false;
}

jit_register64 ir_compiler::get_temp_register(const ir_data_flow_pair& data, const ir_storage_type& storage,
                                              const jit_register64& preferred, const vector<jit_register64>& occupied) {
    unordered_set<jit_register64> live;
    live.insert(occupied.begin(), occupied.end());
    for (const auto& var : data.data_out.live_vars) {
        if (get_storage_type(var.type) != storage) continue;
        auto loc = get_location(var);
        if (loc.reg == no_register) continue;
        live.insert(loc.reg);
    }
    if (preferred != no_register && live.find(preferred) == live.end())
        return preferred;

    const auto& reg_list = storage == ir_storage_type::ir_int ? int_reg_list :
            storage == ir_storage_type::ir_float ? float_reg_list :
            throw runtime_error("Unexpected storage type");
    for (const auto& reg : reg_list) {
        if (reg == no_register) continue;
        if (live.find(reg) == live.end())
            return reg;
    }
    throw runtime_error("Failed to find a free temporary register");
}

const uint8_t* ir_compiler::compile_ssa() {
    cout << endl << "---- x86-64 assembler code ----" << endl;
    assert(blocks[0].label.id == 0) // must start from root block
    for (int i = 0; i < blocks.size(); i++) {
        const auto& block = blocks[i];
        builder.mark_label(block.label.id);
        for (int j = 0; j < block.ir().size(); j++) {
            const auto& [item, data_flow_holder_in] = block.ir()[j];
            auto data_flow_info = ir_data_flow_pair(
                    data_flow_holder_in,
                    j == block.ir().size() - 1 ? block.out_data_flow : block.ir()[j + 1].second
            );
            switch (item->tag) {
                case ir_instruction_tag::load_argument: {
                    // Value is already loaded into the target register, no code is needed
                    break;
                }
                case ir_instruction_tag::assign: {
                    auto instruction = static_pointer_cast<ir_assign_instruction>(item);
                    auto to = get_location(instruction->to);
                    auto from_value = instruction->from;
                    compile_assign(from_value, to);
                    break;
                }
                case ir_instruction_tag::bin_op: {
                    auto instruction = static_pointer_cast<ir_bin_op_insruction>(item);
                    compile_bin_op(instruction, data_flow_info);
                    break;
                }
                case ir_instruction_tag::convert: {
                    auto instruction = static_pointer_cast<ir_convert_instruction>(item);
                    auto from = instruction->from;
                    auto to_loc = get_location(instruction->to);
                    if (from.mode != ir_value_mode::var) {
                        assert(false)
                    }
                    auto from_loc = get_location(from.var);

                    switch (instruction->mode) {
                        case ir_convert_mode::i2l: {
                            assert(from_loc.bit_size == 32)
                            builder.movsx(from_loc, to_loc);
                            break;
                        }
                        case ir_convert_mode::l2i: {
                            compile_assign(from, to_loc);
                            break;
                        }
                        default_fail
                    }
                    break;
                }
                case ir_instruction_tag::cmp_jump: {
                    auto instruction = static_pointer_cast<ir_cmp_jump_instruction>(item);
                    auto first = instruction->first;
                    auto second = instruction->second;
                    ir_variable_type compare_type;
                    // TODO implement cmp with imm
                    if (first.mode == ir_value_mode::var) {
                        compare_type = first.var.type;
                        auto first_loc = get_location(first.var);
                        switch (second.mode) {
                            case ir_value_mode::var: {
                                auto second_loc = get_location(second.var);
                                switch (first.var.type) {
                                    case ir_variable_type::ir_int: // TODO correct support for int32 cmp
                                    case ir_variable_type::ir_long:
                                        builder.cmp(first_loc, second_loc);
                                        break;
                                    default_fail
                                }
                                break;
                            }
                            case ir_value_mode::int64: {
                                auto second_value = second.int64_value;
                                assert_fit_int32(second_value)
                                builder.cmp(first_loc, second_value);
                                break;
                            }
                            case ir_value_mode::float64: {
                                auto second_value = second.float64_value;
                                builder.comisd(first_loc, second_value);
                                break;
                            }
                            default_fail
                        }
                    } else {
                        assert(false);
                    }
                    bool true_has_phi = has_actual_phi_assigns(*block_map[instruction->label_true], block.label);
                    ir_label true_label = true_has_phi ? create_label() : instruction->label_true;
                    switch (compare_type) {
                        case ir_variable_type::ir_int:
                        case ir_variable_type::ir_long: {
                            switch (instruction->mode) {
                                case ir_cmp_mode::eq: builder.je(true_label.id); break;
                                case ir_cmp_mode::neq: builder.jne(true_label.id); break;
                                case ir_cmp_mode::lt: builder.jl(true_label.id); break;
                                case ir_cmp_mode::le: builder.jle(true_label.id); break;
                                case ir_cmp_mode::gt: builder.jg(true_label.id); break;
                                case ir_cmp_mode::ge: builder.jge(true_label.id); break;
                                default_fail
                            }
                            break;
                        }
                        case ir_variable_type::ir_double: {
                            switch (instruction->mode) {
                                case ir_cmp_mode::lt: builder.jb(true_label.id); break;
                                case ir_cmp_mode::gt: builder.ja(true_label.id); break;
                                default_fail
                            }
                            break;
                        }
                        default_fail
                    }
                    compile_phi_before_jump(block.label, block_map[instruction->label_false]);
                    if (true_has_phi || i >= blocks.size() - 1 || blocks[i + 1].label != instruction->label_false) {
                        builder.jmp(instruction->label_false.id);
                    }
                    if (true_has_phi) {
                        builder.mark_label(true_label.id);
                        compile_phi_before_jump(block.label, block_map[instruction->label_true]);
                        builder.jmp(instruction->label_true.id);
                    }
                    break;
                }
                case ir_instruction_tag::jump: {
                    assert(j == block.ir().size() - 1)
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
                            auto var = instruction->value.var;
                            auto from = get_location(var);
                            jit_value_location return_loc;
                            switch (get_storage_type(var.type)) {
                                case ir_storage_type::ir_int: return_loc = jit_value_location(rax, from.bit_size); break;
                                case ir_storage_type::ir_float: return_loc = jit_value_location(xmm0, from.bit_size); break;
                                default_fail
                            }
                            if (from != return_loc) {
                                compile_assign(from, return_loc);
                            }
                            break;
                        }
                        case ir_value_mode::int64: {
                            compile_assign(instruction->value, rax);
                            break;
                        }
                        case ir_value_mode::float64: {
                            compile_assign(instruction->value, xmm0);
                            break;
                        }
                        default_fail
                    }
                    builder.ret();
                    break;
                }
                case ir_instruction_tag::phi: {
                    break;
                }
                default_fail
            }
        }
    }

    builder.resolve_labels();
    builder.link_constants();

    auto code_chunk = manager.add_code_chunk(builder.get_code());

    cout << endl << "---- Compiled bytes ----" << endl;
    for (const auto& byte : builder.get_code()) {
        printf("%02X ", byte);
    }
    cout << endl << endl;

    return code_chunk;
}

void ir_compiler::compile_assign(const jit_value_location& from, const jit_value_location& to) {
    if (from == to) return;
    auto storage = get_storage_type(from);
    assert(storage == get_storage_type(to));
    switch (storage) {
        case ir_storage_type::ir_int: {
            builder.mov(from, to);
            break;
        }
        case ir_storage_type::ir_float: {
            builder.movsd(from, to);
            break;
        }
        default_fail
    }
}

void ir_compiler::compile_assign(const ir_value& from_value, const jit_value_location& to) {
    switch (from_value.mode) {
        case ir_value_mode::var: {
            auto from = get_location(from_value.var);
            compile_assign(from, to);
            break;
        }
        case ir_value_mode::int64: {
            // TODO support ir_value_mode::int32
            switch (to.bit_size) {
                case 32: builder.mov(static_cast<int32_t>(from_value.int64_value), to); break;
                case 64: builder.mov(from_value.int64_value, to); break;
                default_fail
            }
            break;
        }
        case ir_value_mode::float64: {
            builder.movsd(from_value.float64_value, to);
            break;
        }
        default_fail
    }
}

const uint8_t* ir_compiler::compile() {
    convert_to_ssa();
    optimize();
    build_control_flow_graph();
    color_variables();
    return compile_ssa();
}
