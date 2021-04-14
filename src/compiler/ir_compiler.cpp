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

    unordered_map<int, ir_variable_type> all_var_ids;
    for (const auto& block : blocks) {
        for (int i = 0; i < block.ir.size(); i++) {
            for (const auto& value : block.ir[i]->get_in_values()) {
                if (value->mode != ir_value_mode::var) continue;
                all_var_ids[value->var.id] = value->var.type;
            }
            for (const auto& var : block.ir[i]->get_out_variables()) {
                all_var_ids[var->id] = var->type;
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

    for (auto& block : blocks) {
        for (const auto& [var_id, type] : all_var_ids) {
            vector<pair<ir_label, ir_value>> edges;
            for (const auto& from_label : control_flow_in[block.label]) {
                edges.emplace_back(from_label, ir_variable(var_id, 0, type));
            }
            block.ir.insert(block.ir.begin(), make_shared<ir_phi_instruction>(edges, ir_variable(var_id, 0, type)));
        }
    }

    for (auto& block : blocks) {
        for (const auto& instruction : block.ir) {
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
            if (instruction->edges.size() == 1) {
                block.ir[i] = make_shared<ir_assign_instruction>(instruction->edges[0].second, instruction->to);
                changed = true;
            } else if (instruction->edges.size() > 1) {
                optional<ir_value> unique;
                for (const auto& [_, value] : instruction->edges) {
                    if (value.mode == ir_value_mode::var && instruction->to == value.var) continue;
                    if (unique.has_value()) goto skip;
                    unique = value;
                }
                assert(unique.has_value())
                block.ir[i] = make_shared<ir_assign_instruction>(*unique, instruction->to);
                changed = true;
                skip:;
            }
        }
    }
    return changed;
}

bool simplify_instructions(vector<ir_basic_block>& blocks) {
    bool changed = false;
    for (auto& block : blocks) {
        for (int i = 0; i < block.ir.size(); i++) {
            for (const auto& item : block.ir[i]->get_in_values()) {
                if (item->mode != ir_value_mode::int64) goto ir_loop_end;
            }
            switch (block.ir[i]->tag) {
                case ir_instruction_tag::bin_op: {
                    auto instruction = static_pointer_cast<ir_bin_op_insruction>(block.ir[i]);
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
                    block.ir[i] = make_shared<ir_assign_instruction>(ir_value(value), instruction->to);
                    changed = true;
                    break;
                }
                case ir_instruction_tag::cmp_jump: {
                    auto instruction = static_pointer_cast<ir_cmp_jump_instruction>(block.ir[i]);
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
                    block.ir[i] = make_shared<ir_jump_instruction>(label);
                    changed = true;
                    break;
                }
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
    unordered_set<ir_label> used_labels;
    used_labels.insert(ir_label(0)); // root block
    for (const auto& block : blocks) {
        for (const auto& label : block.ir.back()->get_jump_labels()) {
            used_labels.insert(label);
        }
    }
    auto new_end = remove_if(blocks.begin(), blocks.end(), [&](const auto& block){
        return used_labels.find(block.label) == used_labels.end();
    });
    if (new_end == blocks.end())
        return false;

    blocks.erase(new_end, blocks.end());
    return true;
}

bool combine_compare_jump(vector<ir_basic_block>& blocks) {
    bool changed = false;
    unordered_map<ir_variable, tuple<ir_value, ir_value, ir_cmp_nan_mode>> compare_assigns;
    for (const auto& block : blocks) {
        for (const auto& item : block.ir) {
            if (item->tag != ir_instruction_tag::bin_op) continue;
            auto instruction = static_pointer_cast<ir_bin_op_insruction>(item);
            if (instruction->op != ir_bin_op::cmp) continue;
            compare_assigns.insert_or_assign(instruction->to, make_tuple(instruction->first, instruction->second, instruction->nan_mode));
        }
    }

    for (auto& block : blocks) {
        for (int i = 0; i < block.ir.size(); i++) {
            const auto& item = block.ir[i];
            if (item->tag != ir_instruction_tag::cmp_jump) continue;
            auto instruction = static_pointer_cast<ir_cmp_jump_instruction>(item);
            if (instruction->first.mode != ir_value_mode::var) continue;
            auto iter = compare_assigns.find(instruction->first.var);
            if (iter == compare_assigns.end()) continue;
            auto [new_first, new_second, nan_mode] = iter->second;
            block.ir[i] = make_shared<ir_cmp_jump_instruction>(new_first, new_second, instruction->mode,
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
    auto& last = block->ir[block->ir.size() - 1];
    if (last->tag == ir_instruction_tag::jump && static_pointer_cast<ir_jump_instruction>(last)->label == while_start) {
        return label;
    }
    for (const auto& jump_label : last->get_jump_labels()) {
        auto result = find_while_end(block_map, visited, jump_label, while_start);
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
    auto& last = block->ir[block->ir.size() - 1];
    for (const auto& jump_label : last->get_jump_labels()) {
        if (last->tag == ir_instruction_tag::cmp_jump) {
            auto result = find_while_end(block_map, visited, jump_label, label);
            if (result.has_value()) return make_tuple(label, jump_label, *result);
        }
        auto result = find_while_loop(block_map, visited, jump_label);
        if (result.has_value()) return result;
    }
    return nullopt;
}

void rename_variables(unordered_map<ir_label, ir_basic_block*>& block_map, unordered_set<ir_label>& visited,
                      unordered_map<ir_variable, ir_variable>& rename_map, const ir_label& label, const ir_label& ignore_label) {
    if (visited.find(label) != visited.end()) return;
    visited.insert(label);
    auto block = block_map[label];
    for (auto& item : block->ir) {
        for (auto value : item->get_in_values()) {
            if (value->mode != ir_value_mode::var) continue;
            auto iter = rename_map.find(value->var);
            if (iter == rename_map.end()) continue;
            value->var = iter->second;
        }
    }
    auto& last = block->ir[block->ir.size() - 1];
    for (const auto& jump_label : last->get_jump_labels()) {
        if (jump_label == ignore_label) continue;
        rename_variables(block_map, visited, rename_map, jump_label, ignore_label);
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
        for (const auto& jump_label : block.ir[block.ir.size() - 1]->get_jump_labels()) {
            if (jump_label == loop_first) return false;
        }
    }

    auto while_start_block = temp_block_map[while_start];
    auto loop_first_block = temp_block_map[loop_first];
    auto while_end_block = temp_block_map[while_end];
    for (auto iter = while_start_block->ir.rbegin(); iter != while_start_block->ir.rend(); ++iter) {
        if (!(*iter)->get_jump_labels().empty()) continue;
        loop_first_block->ir.insert(loop_first_block->ir.begin(), (*iter)->clone());
    }
    unordered_map<ir_variable, ir_variable> rename_map;
    for (int i = 0; i < while_start_block->ir.size(); i++) {
        const auto& item = while_start_block->ir[i];
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
        while_start_block->ir[i] = make_shared<ir_phi_instruction>(new_edges, new_var);
    }
    for (int i = 0; i < loop_first_block->ir.size(); i++) {
        const auto& item = loop_first_block->ir[i];
        if (item->tag != ir_instruction_tag::phi) continue;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        vector<pair<ir_label, ir_value>> new_edges;
        for (const auto& [label, value] : instruction->edges) {
            if (label != while_end) continue;
            new_edges.emplace_back(label, value);
            break;
        }
        new_edges.emplace_back(while_start, ir_variable(instruction->to.id, last_var_version[instruction->to.id], instruction->to.type));
        loop_first_block->ir[i] = make_shared<ir_phi_instruction>(new_edges, instruction->to);
    }
    {
        assert(while_end_block->ir[while_end_block->ir.size() - 1]->tag == ir_instruction_tag::jump)
        auto last_instruction = while_start_block->ir[while_start_block->ir.size() - 1]->clone();
        auto last_jump = static_pointer_cast<ir_cmp_jump_instruction>(last_instruction);
        last_jump->mode = negate_cmp_mode(last_jump->mode);
        swap(last_jump->label_true, last_jump->label_false);
        while_end_block->ir[while_end_block->ir.size() - 1] = last_jump;
    }
    unordered_map<ir_variable, ir_value> while_end_last_var_map;
    for (const auto& item : loop_first_block->ir) {
        if (item->tag != ir_instruction_tag::phi) continue;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        for (const auto& [label, value] : instruction->edges) {
            if (label != while_end) continue;
            while_end_last_var_map.insert_or_assign(instruction->to, value);
        }
    }
    auto new_jump = while_end_block->ir[while_end_block->ir.size() - 1];
    for (auto value : new_jump->get_in_values()) {
        if (value->mode != ir_value_mode::var) continue;
        auto iter = while_end_last_var_map.find(value->var);
        if (iter == while_end_last_var_map.end()) continue;
        *value = iter->second;
    }

    visited.clear();
    rename_variables(temp_block_map, visited, rename_map, while_start, loop_first);
    ir_label after_label;
    for (const auto& jump_label : while_start_block->ir[while_start_block->ir.size() - 1]->get_jump_labels()) {
        if (jump_label == loop_first) continue;
        after_label = jump_label;
        break;
    }
    assert(after_label.id >= 0)
    auto after_block = temp_block_map[after_label];

    unordered_map<ir_variable, ir_value> reassign_map;
    for (const auto& item : loop_first_block->ir) {
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
    after_block->ir.insert(after_block->ir.begin(), to_add.begin(), to_add.end());

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
                                  unordered_map<jit_value_location, jit_value_location>& temp) {
    auto iter = visited_version.find(start);
    if (iter != visited_version.end()) {
        auto found_version = iter->second;
        if (found_version != version) return;
        // loop detected, TODO properly allocate a temp register
        compile_assign(start, r11);
        temp[start] = r11;
        return;
    }
    visited_version[start] = version;
    auto to_iter = assign_from_map.find(start);
    if (to_iter == assign_from_map.end()) return;
    for (const auto& to : to_iter->second) {
        compile_phi_dfs(to, version, assign_from_map, visited_version, temp);
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

    for (const auto& item : target_block->ir) {
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
        compile_phi_dfs(from, version, assign_from_map, visited_version, temp);
        version++;
    }
    for (const auto& [to, value] : not_var_assign_list) {
        compile_assign(value, to);
    }
}

bool ir_compiler::has_actual_phi_assigns(const ir_basic_block& block, const ir_label& from) {
    for (const auto& item : block.ir) {
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

const uint8_t* ir_compiler::compile_ssa() {
    cout << endl << "---- x86-64 assembler code ----" << endl;
    assert(blocks[0].label.id == 0) // must start from root block
    for (int i = 0; i < blocks.size(); i++) {
        const auto& block = blocks[i];
        builder.mark_label(block.label.id);
        for (int j = 0; j < block.ir.size(); j++) {
            const auto& item = block.ir[j];
            switch (item->tag) {
                case ir_instruction_tag::assign: {
                    auto instruction = static_pointer_cast<ir_assign_instruction>(item);
                    auto to = get_location(instruction->to);
                    auto from_value = instruction->from;
                    compile_assign(from_value, to);
                    break;
                }
                case ir_instruction_tag::bin_op: {
                    auto instruction = static_pointer_cast<ir_bin_op_insruction>(item);
                    compile_bin_op(instruction);
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
                default: assert(false)
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
            builder.mov(from_value.int64_value, to);
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
    color_variables();
    return compile_ssa();
}
