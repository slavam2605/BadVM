#include "ir_compiler.h"
#include "../utils/utils.h"
#include "code_builder.h"
#include "register_shortcuts.h"
#include "code_manager.h"
#include <unordered_set>
#include <queue>

#define assert_fit_int32(x) assert(static_cast<int32_t>(x) == x)

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
            if (instruction->edges.size() != 1) continue;
            block.ir[i] = make_shared<ir_assign_instruction>(instruction->edges[0].second, instruction->to);
            changed = true;
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
                    auto first = instruction->first.value;
                    auto second = instruction->second.value;
                    int64_t value;
                    switch (instruction->op) {
                        case ir_bin_op::add: value = first + second; break;
                        case ir_bin_op::sub: value = first - second; break;
                        case ir_bin_op::mul: value = first * second; break;
                        default_fail
                    }
                    block.ir[i] = make_shared<ir_assign_instruction>(ir_value(value), instruction->to);
                    changed = true;
                    break;
                }
                case ir_instruction_tag::cmp_jump: {
                    auto instruction = static_pointer_cast<ir_cmp_jump_instruction>(block.ir[i]);
                    auto first = instruction->first.value;
                    auto second = instruction->second.value;
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

void ir_compiler::optimize() {
    bool changed = true;
    while (changed) {
        changed = false;
        changed |= simplify_instructions(blocks);
        changed |= inline_assigns(blocks);
        changed |= eliminate_trivial_phi(blocks);
        changed |= remove_unused_vars(blocks);
        changed |= remove_unused_blocks(blocks);
    }

    cout << endl << "---- Optimized SSA IR ----" << endl;
    for (const auto& block : blocks) {
        pretty_print(cout, block);
    }
}

void ir_compiler::compile_phi_dfs(const jit_register64& start, int version,
                                  const unordered_map<jit_register64, unordered_set<jit_register64>>& assign_from_map,
                                  unordered_map<jit_register64, int>& visited_version) {
    auto iter = visited_version.find(start);
    if (iter != visited_version.end()) {
        auto found_version = iter->second;
        if (found_version != version) return;
        assert(false) // loop found, TODO: support loop resolution
    }
    visited_version[start] = version;
    auto to_iter = assign_from_map.find(start);
    if (to_iter == assign_from_map.end()) return;
    for (const auto& to : to_iter->second) {
        compile_phi_dfs(to, version, assign_from_map, visited_version);
        compile_assign(to, start);
    }
}

void ir_compiler::compile_phi_before_jump(const ir_label& current_label, const ir_basic_block* target_block) {
//    cout << "L" << current_label.id << "_PHI_BLOCK:" << endl;
    unordered_map<jit_register64, unordered_set<jit_register64>> assign_from_map;
    vector<pair<jit_register64, ir_value>> not_var_assign_list;

    for (const auto& item : target_block->ir) {
        if (item->tag != ir_instruction_tag::phi) break; // assume all phi's are in the beginning of a block
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        for (const auto& [a_label, value] : instruction->edges) {
            if (a_label != current_label) continue;
            if (value.mode == ir_value_mode::var) {
                auto from_reg = reg_list[color[value.var]];
                auto to_reg = reg_list[color[instruction->to]];
                if (from_reg != to_reg) {
                    assign_from_map[from_reg].insert(to_reg);
                }
            } else {
                not_var_assign_list.emplace_back(reg_list[color[instruction->to]], value);
            }
            break;
        }
    }

    // TODO add cycle detection and resolution
    int version = 0;
    unordered_map<jit_register64, int> visited_version;
    for (const auto& [from, _] : assign_from_map) {
        compile_phi_dfs(from, version, assign_from_map, visited_version);
        version++;
    }
    for (const auto& [to, value] : not_var_assign_list) {
        compile_assign(to, value);
    }
}

void ir_compiler::compile_bin_op(const shared_ptr<ir_bin_op_insruction>& instruction) {
    auto op = instruction->op;
    auto first_value = instruction->first;
    auto second_value = instruction->second;
    auto to = reg_list[color[instruction->to]];
    switch (op) {
        case ir_bin_op::add:
        case ir_bin_op::mul: {
            if ((first_value.mode != ir_value_mode::var && second_value.mode == ir_value_mode::var) ||
                (first_value.mode == ir_value_mode::var && second_value.mode == ir_value_mode::var &&
                 reg_list[color[second_value.var]] == to)) {
                swap(first_value, second_value);
            }
            break;
        }
        case ir_bin_op::sub: break;
        default_fail
    }

    switch (first_value.mode) {
        case ir_value_mode::var: {
            auto first = reg_list[color[first_value.var]];
            switch (second_value.mode) {
                case ir_value_mode::var: {
                    auto second = reg_list[color[second_value.var]];
                    switch (op) {
                        case ir_bin_op::add: {
                            if (first == to) {
                                builder.add(second, to);
                            } else {
                                assert(second != to)
                                builder.mov(first, to);
                                builder.add(second, to);
                            }
                            break;
                        }
                        case ir_bin_op::sub: {
                            if (first == to) {
                                builder.sub(second, to);
                            } else if (second == to) {
                                // This is faster then `sub to, first; neg to`
                                builder.neg(to);
                                builder.add(first, to);
                            } else {
                                builder.mov(first, to);
                                builder.sub(second, to);
                            }
                            break;
                        }
                        case ir_bin_op::mul: {
                            if (first != to) {
                                builder.mov(first, to);
                            }
                            builder.mul(second, to);
                            break;
                        }
                        default_fail
                    }
                    break;
                }
                case ir_value_mode::int64: {
                    // TODO implement add with int64
                    auto value = second_value.value;
                    assert_fit_int32(value)
                    if (first != to) {
                        builder.mov(first, to);
                    }
                    switch (op) {
                        case ir_bin_op::add: builder.add(value, to); break;
                        case ir_bin_op::sub: assert(false)
                        case ir_bin_op::mul: assert(false)
                        default: assert(false)
                    }
                    break;
                }
                default: assert(false)
            }
            break;
        }
        case ir_value_mode::int64: {
            switch (op) {
                case ir_bin_op::sub: {
                    assert(false) // TODO
                    break;
                }
                default_fail
            }
        }
        default: assert(false)
    }
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
                    auto to = reg_list[color[instruction->to]];
                    auto from_value = instruction->from;
                    compile_assign(to, from_value);
                    break;
                }
                case ir_instruction_tag::bin_op: {
                    auto instruction = static_pointer_cast<ir_bin_op_insruction>(item);
                    compile_bin_op(instruction);
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
                        assert_fit_int32(second_value)
                        builder.cmp(first_reg, second_value);
                    } else {
                        assert(false);
                    }
                    switch (instruction->mode) {
                        case ir_cmp_mode::eq: builder.je(instruction->label_true.id); break;
                        case ir_cmp_mode::neq: builder.jne(instruction->label_true.id); break;
                        case ir_cmp_mode::lt: builder.jl(instruction->label_true.id); break;
                        case ir_cmp_mode::le: builder.jle(instruction->label_true.id); break;
                        case ir_cmp_mode::gt: builder.jg(instruction->label_true.id); break;
                        case ir_cmp_mode::ge: builder.jge(instruction->label_true.id); break;
                    }
                    // TODO insert a new block with phi's impl
                    assert(block_map[instruction->label_true]->ir[0]->tag != ir_instruction_tag::phi)
                    compile_phi_before_jump(block.label, block_map[instruction->label_false]);
                    if (i >= blocks.size() - 1 || blocks[i + 1].label != instruction->label_false) {
                        builder.jmp(instruction->label_false.id);
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

    cout << endl << "---- Compiled bytes ----" << endl;
    for (const auto& byte : builder.get_code()) {
        printf("%02X ", byte);
    }
    cout << endl << endl;

    return code_chunk;
}

void ir_compiler::compile_assign(const jit_register64& to, const jit_register64& from) {
    if (from == to) return;
    builder.mov(from, to);
}

void ir_compiler::compile_assign(const jit_register64& to, const ir_value& from_value) {
    switch (from_value.mode) {
        case ir_value_mode::var: {
            auto from = reg_list[color[from_value.var]];
            compile_assign(to, from);
            break;
        }
        case ir_value_mode::int64: {
            builder.mov(from_value.value, to);
            break;
        }
        default: assert(false)
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

    cout << endl << "---- Variables coloring ----" << endl;
    for (const auto& [var, _] : interference_graph) {
        cout << var << " -> " << color[var] << endl;
    }
}

const uint8_t* ir_compiler::compile() {
    convert_to_ssa();
    optimize();
    color_variables();
    return compile_ssa();
}
