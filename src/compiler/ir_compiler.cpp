#include "ir_compiler.h"
#include "../utils/utils.h"
#include "register_shortcuts.h"
#include <unordered_set>

#define assert_fit_int32(x) assert(static_cast<int32_t>(x) == x)

using namespace std;

ir_compiler::ir_compiler(code_manager& manager)
        : manager(manager), reg_list{jit_register64::no_register, rax, rcx, rdx, r8, r9/*, r10, r11*/} {}

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

    unordered_map<int, int> last_var_version;
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
                                  unordered_map<jit_register64, int>& visited_version,
                                  unordered_map<jit_register64, jit_register64>& temp) {
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
    unordered_map<jit_register64, jit_register64> temp;
    for (const auto& [from, _] : assign_from_map) {
        temp.clear();
        compile_phi_dfs(from, version, assign_from_map, visited_version, temp);
        version++;
    }
    for (const auto& [to, value] : not_var_assign_list) {
        compile_assign(value, to);
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
                    auto to_reg = reg_list[color[instruction->to]];
                    if (from.mode != ir_value_mode::var) {
                        assert(false)
                    }
                    auto from_reg = reg_list[color[from.var]];

                    switch (instruction->mode) {
                        case ir_convert_mode::i2l: {
                            // TODO replace reg_list[color[...]] with a proper way to get a register with fitting size
                            auto from32 = jit_value_location(from_reg, 32);
                            builder.movsx(from32, to_reg);
                            break;
                        }
                        case ir_convert_mode::l2i: {
                            compile_assign(from, to_reg);
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
                    // TODO implement cmp with imm
                    if (first.mode == ir_value_mode::var) {
                        auto first_reg = reg_list[color[first.var]];
                        switch (second.mode) {
                            case ir_value_mode::var: {
                                auto second_reg = reg_list[color[second.var]];
                                builder.cmp(first_reg, second_reg);
                                break;
                            }
                            case ir_value_mode::int64: {
                                auto second_value = second.value;
                                assert_fit_int32(second_value)
                                builder.cmp(first_reg, second_value);
                                break;
                            }
                            default_fail
                        }
                    } else {
                        assert(false);
                    }
                    bool true_has_phi = block_map[instruction->label_true]->ir[0]->tag == ir_instruction_tag::phi;
                    ir_label true_label = true_has_phi ? create_label() : instruction->label_true;
                    switch (instruction->mode) {
                        case ir_cmp_mode::eq: builder.je(true_label.id); break;
                        case ir_cmp_mode::neq: builder.jne(true_label.id); break;
                        case ir_cmp_mode::lt: builder.jl(true_label.id); break;
                        case ir_cmp_mode::le: builder.jle(true_label.id); break;
                        case ir_cmp_mode::gt: builder.jg(true_label.id); break;
                        case ir_cmp_mode::ge: builder.jge(true_label.id); break;
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

void ir_compiler::compile_assign(const jit_register64& from, const jit_register64& to) {
    if (from == to) return;
    builder.mov(from, to);
}

void ir_compiler::compile_assign(const ir_value& from_value, const jit_register64& to) {
    switch (from_value.mode) {
        case ir_value_mode::var: {
            auto from = reg_list[color[from_value.var]];
            compile_assign(from, to);
            break;
        }
        case ir_value_mode::int64: {
            builder.mov(from_value.value, to);
            break;
        }
        default: assert(false)
    }
}

const uint8_t* ir_compiler::compile() {
    convert_to_ssa();
    optimize();
    color_variables();
    return compile_ssa();
}
