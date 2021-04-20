#include "ir_compiler.h"

using namespace std;

bool find_loop(unordered_map<ir_label, ir_basic_block*>& block_map, unordered_set<ir_label>& visited,
               const unordered_set<ir_label>& non_loop, unordered_set<ir_label>& in_loop,
               const ir_label& label, const ir_label& while_start) {
    if (label == while_start) return true;
    if (in_loop.find(label) != in_loop.end()) return true;
    if (non_loop.find(label) != non_loop.end()) return true;
    if (visited.find(label) != visited.end()) return false;
    visited.insert(label);
    const auto& block = block_map[label];
    const auto& [last, _] = block->ir().back();
    auto result = false;
    for (const auto& jump_label : last->get_jump_labels()) {
        if (find_loop(block_map, visited, non_loop, in_loop, *jump_label, while_start)) {
            in_loop.insert(label);
            result = true;
        }
    }
    return result;
}

bool mark_maybe_non_loop(unordered_map<ir_label, ir_basic_block*>& block_map, unordered_set<ir_label>& visited,
                         unordered_set<ir_label>& non_loop, const ir_label& loop_enter, const ir_label& current_label) {
    if (non_loop.find(current_label) != non_loop.end()) return true;
    if (visited.find(current_label) != visited.end()) return false;
    visited.insert(current_label);
    auto current_block = block_map[current_label];
    const auto& [last, _] = current_block->ir().back();
    bool result = false;
    for (const auto& jump_label : last->get_jump_labels()) {
        if (mark_maybe_non_loop(block_map, visited, non_loop, loop_enter, *jump_label) && current_label != loop_enter) {
            non_loop.insert(current_label);
            result = true;
        }
    }
    return result;
}

void mark_non_loop(unordered_map<ir_label, ir_basic_block*>& block_map, unordered_set<ir_label>& visited,
                   unordered_set<ir_label>& non_loop, const ir_label& loop_enter, const ir_label& current_label) {
    if (visited.find(current_label) != visited.end()) return;
    visited.insert(current_label);
    non_loop.insert(current_label);
    auto current_block = block_map[current_label];
    const auto& [last, _] = current_block->ir().back();
    for (const auto& jump_label : last->get_jump_labels()) {
        if (*jump_label == loop_enter) {
            mark_maybe_non_loop(block_map, visited, non_loop, loop_enter, *jump_label);
        } else {
            mark_non_loop(block_map, visited, non_loop, loop_enter, *jump_label);
        }
    }
}

optional<pair<ir_label, unordered_set<ir_label>>> find_while_loop(unordered_map<ir_label, ir_basic_block*>& block_map,
                                                                  unordered_set<ir_label>& visited, const ir_label& label) {
    if (visited.find(label) != visited.end()) return nullopt;
    visited.insert(label);
    auto block = block_map[label];
    auto& [last, _] = block->ir().back();
    if (last->tag == ir_instruction_tag::cmp_jump) {
        unordered_set<ir_label> non_loop_visited, non_loop;
        mark_non_loop(block_map, non_loop_visited, non_loop, label, ir_label(0));

        unordered_set<ir_label> find_loop_visited, in_loop;
        for (const auto& jump_label : last->get_jump_labels()) {
            find_loop(block_map, find_loop_visited, non_loop, in_loop, *jump_label, label);
        }
        if (!in_loop.empty())
            return make_pair(label, in_loop);
    }
    for (const auto& jump_label : last->get_jump_labels()) {
        auto result = find_while_loop(block_map, visited, *jump_label);
        if (result.has_value()) return result;
    }
    return nullopt;
}

void rename_variables(unordered_map<ir_label, ir_basic_block*>& block_map, unordered_set<ir_label>& visited,
                      unordered_map<ir_variable, ir_variable>& rename_map, const ir_label& label,
                      const ir_label& ignore_label, const ir_label& prev_label) {
    if (visited.find(label) != visited.end()) return;
    visited.insert(label);
    auto block = block_map[label];
    unordered_map<ir_variable, ir_variable> removed_renames;
    for (auto& [item, _] : block->ir()) {
        for (const auto& var : item->get_out_variables()) {
            if (rename_map.find(*var) == rename_map.end()) continue;
            auto old_rename = rename_map[*var];
            removed_renames[*var] = old_rename;
            rename_map.erase(*var);
        }

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
        rename_variables(block_map, visited, rename_map, *jump_label, ignore_label, prev_label);
    }

    for (const auto& [from_var, to_var] : removed_renames) {
        rename_map[from_var] = to_var;
    }
}

optional<tuple<ir_label, unordered_set<ir_label>, vector<ir_label>, vector<ir_label>>>
find_while_loop_blocks(unordered_map<ir_label, ir_basic_block*>& block_map) {
    unordered_set<ir_label> visited;
    auto result = find_while_loop(block_map, visited, ir_label(0));
    if (!result.has_value()) return nullopt;
    auto [while_start, loop_blocks] = *result;
    auto while_start_block = block_map[while_start];

    vector<ir_label> loop_first_blocks;
    for (const auto& while_start_next_label : while_start_block->ir().back().first->get_jump_labels()) {
        if (loop_blocks.find(*while_start_next_label) == loop_blocks.end()) continue;
        loop_first_blocks.push_back(*while_start_next_label);
    }

    vector<ir_label> last_goto_blocks;
    for (const auto& label : loop_blocks) {
        auto block = block_map[label];
        auto [last, _] = block->ir().back();
        if (last->tag != ir_instruction_tag::jump) continue;
        auto instruction = static_pointer_cast<ir_jump_instruction>(last);
        if (instruction->label == while_start)
            last_goto_blocks.push_back(label);
    }

    return make_tuple(while_start, loop_blocks, loop_first_blocks, last_goto_blocks);
}

void copy_instructions_to_first_block_in_loop(const ir_basic_block& while_start_block, ir_basic_block& loop_first_block) {
    for (auto iter = while_start_block.ir().rbegin(); iter != while_start_block.ir().rend(); ++iter) {
        if (!iter->first->get_jump_labels().empty()) continue;
        loop_first_block.insert(0, iter->first->clone());
    }
}

unordered_map<ir_variable, ir_variable>
filter_while_start_phi_and_create_rename_map(ir_basic_block& while_start_block, const unordered_set<ir_label>& last_goto_set,
                                             unordered_map<int, int>& last_var_version) {
    unordered_map<ir_variable, ir_variable> rename_map;
    for (int i = 0; i < while_start_block.ir().size(); i++) {
        const auto& [item, _] = while_start_block.ir()[i];
        if (item->tag != ir_instruction_tag::phi) continue;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        vector<pair<ir_label, ir_value>> new_edges;
        for (const auto& [label, value] : instruction->edges) {
            if (last_goto_set.find(label) != last_goto_set.end()) continue;
            new_edges.emplace_back(label, value);
        }
        auto new_version = ++last_var_version[instruction->to.id];
        auto new_var = ir_variable(instruction->to.id, new_version, instruction->to.type);
        rename_map[instruction->to] = new_var;
        while_start_block.emplace_at<ir_phi_instruction>(i, new_edges, new_var);
    }
    return rename_map;
}

void fix_copied_phi_instructions(const ir_label& while_start, ir_basic_block& loop_first_block,
                                 const unordered_set<ir_label>& last_goto_set, unordered_map<int, int>& last_var_version) {
    for (int i = 0; i < loop_first_block.ir().size(); i++) {
        const auto& [item, _] = loop_first_block.ir()[i];
        if (item->tag != ir_instruction_tag::phi) continue;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        vector<pair<ir_label, ir_value>> new_edges;
        for (const auto& [label, value] : instruction->edges) {
            if (last_goto_set.find(label) == last_goto_set.end()) continue;
            new_edges.emplace_back(label, value);
            break;
        }
        new_edges.emplace_back(while_start, ir_variable(instruction->to.id, last_var_version[instruction->to.id], instruction->to.type));
        loop_first_block.emplace_at<ir_phi_instruction>(i, new_edges, instruction->to);
    }
}

void replace_last_loop_jumps(unordered_map<ir_label, ir_basic_block*> block_map, const vector<ir_label>& last_goto_labels,
                             const ir_basic_block& while_start_block) {
    for (const auto& last_goto_label : last_goto_labels) {
        auto last_goto_block = block_map[last_goto_label];
        auto last_instruction = while_start_block.ir().back().first->clone();
        auto last_jump = static_pointer_cast<ir_cmp_jump_instruction>(last_instruction);
        last_jump->mode = negate_cmp_mode(last_jump->mode);
        swap(last_jump->label_true, last_jump->label_false);
        last_goto_block->set(last_goto_block->ir().size() - 1, last_jump);
    }
}

void fix_last_loop_jump_variables(unordered_map<ir_label, ir_basic_block*> block_map,
                                  const ir_basic_block& loop_first_block, const vector<ir_label>& last_goto_labels) {
    for (const auto& last_goto_label : last_goto_labels) {
        unordered_map<ir_variable, ir_value> while_end_last_var_map;
        for (const auto& [item, _] : loop_first_block.ir()) {
            if (item->tag != ir_instruction_tag::phi) continue;
            auto instruction = static_pointer_cast<ir_phi_instruction>(item);
            for (const auto& [label, value] : instruction->edges) {
                if (label != last_goto_label) continue;
                while_end_last_var_map.insert_or_assign(instruction->to, value);
            }
        }
        auto[new_jump, _] = block_map[last_goto_label]->ir().back();
        for (auto value : new_jump->get_in_values()) {
            if (value->mode != ir_value_mode::var) continue;
            auto iter = while_end_last_var_map.find(value->var);
            if (iter == while_end_last_var_map.end()) continue;
            *value = iter->second;
        }
    }
}

unordered_set<ir_label> find_loop_after_labels(const ir_basic_block& while_start_block, const ir_label& loop_first) {
    unordered_set<ir_label> after_labels;
    for (const auto& jump_label : while_start_block.ir().back().first->get_jump_labels()) {
        if (*jump_label == loop_first) continue;
        after_labels.insert(*jump_label);
    }
    return after_labels;
}

ir_label ir_compiler::create_new_block(const ir_label& position, const ir_label& goto_block) {
    auto new_label = create_label();
    ir_basic_block block(new_label);
    block.emplace_back<ir_jump_instruction>(goto_block);
    for (int i = 0; i < blocks.size(); i++) {
        if (blocks[i].label != position) continue;
        blocks.insert(blocks.begin() + i + 1, block);
        break;
    }
    return new_label;
}

void fix_while_start_jump(ir_basic_block& while_start_block, const ir_label& old_loop_first, const ir_label& new_loop_start) {
    auto jump = while_start_block.ir().back().first;
    for (auto jump_label : jump->get_jump_labels()) {
        if (*jump_label == old_loop_first) {
            *jump_label = new_loop_start;
        }
    }
}

void fix_phi_in_old_first_block(ir_basic_block& old_first_block, const ir_label& while_start, const ir_label& new_loop_start) {
    for (auto& [item, _] : old_first_block.ir()) {
        if (item->tag != ir_instruction_tag::phi) continue;
        auto instruction = static_pointer_cast<ir_phi_instruction>(item);
        for (auto& [label, _] : instruction->edges) {
            if (label == while_start) {
                label = new_loop_start;
            }
        }
    }
}

bool ir_compiler::inverse_loops() {
    unordered_map<ir_label, ir_basic_block*> temp_block_map;
    for (auto& block : blocks) {
        temp_block_map[block.label] = &block;
    }

    auto result = find_while_loop_blocks(temp_block_map);
    if (!result.has_value()) return false;
    auto [while_start, loop_labels, loop_first_labels, last_goto_labels] = *result;
    unordered_set<ir_label> last_goto_set(last_goto_labels.begin(), last_goto_labels.end());

    if (loop_first_labels.size() != 1) return false;
    auto new_loop_start = create_new_block(while_start, loop_first_labels[0]);

    temp_block_map.clear();
    for (auto& block : blocks) {
        temp_block_map[block.label] = &block;
    }
    auto while_start_block = temp_block_map[while_start];
    auto new_loop_start_block = temp_block_map[new_loop_start];

    fix_while_start_jump(*while_start_block, loop_first_labels[0], new_loop_start);
    fix_phi_in_old_first_block(*temp_block_map[loop_first_labels[0]], while_start, new_loop_start);
    copy_instructions_to_first_block_in_loop(*while_start_block, *new_loop_start_block);
    auto rename_map = filter_while_start_phi_and_create_rename_map(*while_start_block, last_goto_set, last_var_version);
    fix_copied_phi_instructions(while_start, *new_loop_start_block, last_goto_set, last_var_version);
    replace_last_loop_jumps(temp_block_map, last_goto_labels, *while_start_block);
    fix_last_loop_jump_variables(temp_block_map, *new_loop_start_block, last_goto_labels);

    unordered_set<ir_label> visited;
    rename_variables(temp_block_map, visited, rename_map, while_start, new_loop_start, ir_label(-1));

    auto after_labels = find_loop_after_labels(*while_start_block, new_loop_start);
    assert(after_labels.size() == 1)
    auto after_label = *after_labels.begin();
    auto after_block = temp_block_map[after_label];

    unordered_map<ir_variable, vector<pair<ir_label, ir_value>>> reassign_map;
    for (const auto& [item, _] : new_loop_start_block->ir()) {
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
        for (const auto& last_goto_label : last_goto_labels) {
            optional<ir_value> from_while_end;
            for (const auto& [label, value] : instruction->edges) {
                if (label != last_goto_label) continue;
                from_while_end = value;
                break;
            }
            assert(from_while_end.has_value())
            reassign_map[*from_while_start].emplace_back(last_goto_label, *from_while_end);
        }
    }

    unordered_map<ir_variable, ir_variable> new_rename_map;
    vector<shared_ptr<ir_instruction>> to_add;
    for (const auto& [from_start, pairs] : reassign_map) {
        auto new_version = ++last_var_version[from_start.id];
        auto new_var = ir_variable(from_start.id, new_version, from_start.type);
        new_rename_map[from_start] = new_var;
        vector<pair<ir_label, ir_value>> new_edges;
        new_edges.emplace_back(while_start, from_start);
        new_edges.insert(new_edges.end(), pairs.begin(), pairs.end());
        auto new_phi = make_shared<ir_phi_instruction>(new_edges, new_var);
        to_add.push_back(new_phi);
    }

    visited.clear();
    rename_variables(temp_block_map, visited, new_rename_map, after_label, ir_label(), ir_label(-1));
    after_block->insert_all(0, to_add);
    return true;
}