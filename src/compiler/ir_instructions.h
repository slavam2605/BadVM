#ifndef BADJVM_IR_INSTRUCTIONS_H
#define BADJVM_IR_INSTRUCTIONS_H

#include <cstdint>
#include <functional>
#include <algorithm>
#include <ostream>

struct ir_variable {
    int id, version;

    ir_variable() : id(-1), version(-1) {}
    explicit ir_variable(int id) : id(id), version(0) {}
    ir_variable(int id, int version) : id(id), version(version) {}

    bool operator==(const ir_variable& rhs) const { return id == rhs.id && version == rhs.version; }
    bool operator!=(const ir_variable& rhs) const { return !(rhs == *this); }
};

std::ostream& operator<<(std::ostream& stream, const ir_variable& var);

enum class ir_value_mode {
    var, int64
};

struct ir_value {
    ir_value_mode mode;
    ir_variable var;
    int64_t value;

    ir_value(const ir_variable& var) : mode(ir_value_mode::var), var(var) {}
    explicit ir_value(int64_t value) : mode(ir_value_mode::int64), value(value) {}
};

std::ostream& operator<<(std::ostream& stream, const ir_value& var);

struct ir_label {
    int id;

    ir_label() : id(-1) {}
    explicit ir_label(int id) : id(id) {}

    bool operator==(const ir_label& rhs) const { return id == rhs.id; }
    bool operator!=(const ir_label& rhs) const { return !(rhs == *this); }
};

namespace std {
    template <>
    struct hash<ir_label> {
        size_t operator()(const ir_label& label) const {
            return label.id;
        }
    };

    template <>
    struct hash<ir_variable> {
        size_t operator()(const ir_variable& var) const {
            return var.id * 10000 + var.version;
        }
    };
}

enum class ir_instruction_tag {
    assign, bin_op, cmp_jump, jump, ret, phi
};

struct ir_instruction {
    const ir_instruction_tag tag;

    ir_instruction(const ir_instruction_tag tag) : tag(tag) {}
    virtual std::vector<ir_label> get_jump_labels() const { return {}; }
    virtual bool exit_function() const { return false; }
    virtual std::vector<ir_value*> get_in_values() { return {}; }
    virtual std::vector<ir_variable*> get_out_variables() { return {}; };
    virtual bool is_pure() const { return true; }
};

struct ir_assign_instruction : ir_instruction {
    ir_value from;
    ir_variable to;

    ir_assign_instruction(const ir_value& from, const ir_variable& to) : ir_instruction(ir_instruction_tag::assign), from(from), to(to) {}
    std::vector<ir_value*> get_in_values() override { if (from.mode == ir_value_mode::var) return {&from}; else return {}; }
    std::vector<ir_variable*> get_out_variables() override { return {&to}; }
};

enum class ir_bin_op {
    add, sub
};

struct ir_bin_op_insruction : ir_instruction {
    ir_value first, second;
    ir_variable to;
    ir_bin_op op;

    ir_bin_op_insruction(const ir_value& first, const ir_value& second, const ir_variable& to, const ir_bin_op& op)
            : ir_instruction(ir_instruction_tag::bin_op), first(first), second(second), to(to), op(op) {}

    std::vector<ir_value*> get_in_values() override { return {&first, &second}; }
    std::vector<ir_variable*> get_out_variables() override { return {&to}; }
};

enum class ir_cmp_mode {
    eq, neq, lt, le, gt, ge
};

struct ir_cmp_jump_instruction : ir_instruction {
    ir_value first, second;
    ir_cmp_mode mode;
    ir_label label_true, label_false;

    ir_cmp_jump_instruction(const ir_value& first, const ir_value& second, ir_cmp_mode mode,
                            const ir_label& label_true, const ir_label& label_false)
            : ir_instruction(ir_instruction_tag::cmp_jump), first(first), second(second),
              mode(mode), label_true(label_true), label_false(label_false) {}

    std::vector<ir_label> get_jump_labels() const override { return {label_true, label_false}; }
    std::vector<ir_value*> get_in_values() override { return {&first, &second}; }
};

struct ir_jump_instruction : ir_instruction {
    ir_label label;

    ir_jump_instruction(const ir_label& label) : ir_instruction(ir_instruction_tag::jump), label(label) {}
    std::vector<ir_label> get_jump_labels() const override { return {label}; }
};

struct ir_ret_instruction : ir_instruction {
    ir_value value;

    ir_ret_instruction(const ir_value& value) : ir_instruction(ir_instruction_tag::ret), value(value) {}
    bool exit_function() const override { return true; }
    std::vector<ir_value*> get_in_values() override { return {&value}; }
};

struct ir_phi_instruction : ir_instruction {
    std::vector<std::pair<ir_label, ir_value>> edges;
    ir_variable to;

    ir_phi_instruction(const std::vector<std::pair<ir_label, ir_value>>& edges, const ir_variable& to)
            : ir_instruction(ir_instruction_tag::phi), edges(edges), to(to) {}

    std::vector<ir_value*> get_in_values() override {
        std::vector<ir_value*> result;
        std::transform(edges.begin(), edges.end(), std::back_inserter(result),
                       [](auto& item){ return &item.second; });
        return result;
    }
    std::vector<ir_variable*> get_out_variables() override { return {&to}; }
};

#endif //BADJVM_IR_INSTRUCTIONS_H
