#include "ir_compiler.h"
#include <queue>

using namespace std;

template <class State, class Getter, class Merge>
inline State merge_succ_blocks(ir_compiler& compiler, const ir_basic_block& block, const Merge& merge, const Getter& getter) {
    State state;
    for (const auto& succ_label : compiler.control_flow_out[block.label]) {
        auto succ_block = compiler.block_map[succ_label];
        const auto& succ_in_state = getter(succ_block->ir()[0].second);
        merge(state, block, *succ_block, succ_in_state);
    }
    return state;
}

template <class State, class Transfer, class Merge, class Getter, class Setter, class Equal>
inline void backward_analysis(ir_compiler& compiler, const Transfer& transfer, const Merge& merge, const Getter& getter, const Setter& setter, const Equal& equal) {
    queue<ir_label> work_list;
    unordered_set<ir_label> in_list;
    for (auto iter = compiler.blocks.rbegin(); iter != compiler.blocks.rend(); ++iter) {
        work_list.push(iter->label);
        in_list.insert(iter->label);
    }

    while (!work_list.empty()) {
        auto label = work_list.front();
        work_list.pop();
        in_list.erase(label);
        auto block = compiler.block_map[label];
        auto state = merge_succ_blocks<State>(compiler, *block, merge, getter);

        auto old_in_state = block->ir()[0].second;
        for (int i = block->ir().size() - 1; i >= 0; i--) {
            const auto& [instruction, _] = block->ir()[i];
            if (i == block->ir().size() - 1)
                setter(block->out_data_flow, state);
            else
                setter(block->_ir[i + 1].second, state);
            transfer(state, instruction);
            setter(block->_ir[i].second, state);
        }
        if (equal(block->ir()[0].second, old_in_state))
            continue;

        for (const auto& pred_label : compiler.control_flow_in[label]) {
            if (in_list.find(pred_label) == in_list.end()) {
                work_list.push(pred_label);
                in_list.insert(pred_label);
            }
        }
    }


}