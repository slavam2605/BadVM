#ifndef BADJVM_IR_INSTRUCTIONS_H
#define BADJVM_IR_INSTRUCTIONS_H

#include <cstdint>
#include <functional>
#include <algorithm>
#include <ostream>
#include <memory>
#include "../utils/utils.h"

enum class ir_variable_type {
    ir_no_type, ir_int, ir_long, ir_double
};

enum class ir_storage_type {
    ir_int, ir_float
};

inline ir_storage_type get_storage_type(const ir_variable_type& type) {
    switch (type) {
        case ir_variable_type::ir_int: return ir_storage_type::ir_int;
        case ir_variable_type::ir_long: return ir_storage_type::ir_int;
        case ir_variable_type::ir_double: return ir_storage_type::ir_float;
        default_fail
    }
}

struct ir_variable {
    int id, version;
    ir_variable_type type;

    ir_variable() : id(-1), version(-1), type(ir_variable_type::ir_no_type) {}
    explicit ir_variable(int id, ir_variable_type type) : id(id), version(0), type(type) {}
    ir_variable(int id, int version, ir_variable_type type) : id(id), version(version), type(type) {}

    bool operator==(const ir_variable& rhs) const { return id == rhs.id && version == rhs.version; }
    bool operator!=(const ir_variable& rhs) const { return !(rhs == *this); }
};

std::ostream& operator<<(std::ostream& stream, const ir_variable& var);

enum class ir_value_mode {
    var, int64, float64
};

struct ir_value {
    ir_value_mode mode;
    ir_variable var;
    int64_t int64_value;
    double float64_value;

    ir_value(const ir_variable& var) : mode(ir_value_mode::var), var(var) {}
    explicit ir_value(int64_t value) : mode(ir_value_mode::int64), int64_value(value) {}
    explicit ir_value(double value) : mode(ir_value_mode::float64), float64_value(value) {}
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
    assign, bin_op, cmp_jump, convert, jump, ret, phi
};

struct ir_instruction {
    const ir_instruction_tag tag;

    ir_instruction(const ir_instruction_tag tag) : tag(tag) {}
    virtual std::shared_ptr<ir_instruction> clone() const = 0;
    virtual std::vector<ir_label> get_jump_labels() const { return {}; }
    virtual bool exit_function() const { return false; }
    virtual std::vector<ir_value*> get_in_values() { return {}; }
    virtual std::vector<ir_variable*> get_out_variables() { return {}; };
    virtual bool is_pure() const { return true; }
};

struct ir_assign_instruction : ir_instruction {
    ir_value from;
    ir_variable to;

    std::shared_ptr<ir_instruction> clone() const override {
        return std::make_shared<ir_assign_instruction>(from, to);
    }
    ir_assign_instruction(const ir_value& from, const ir_variable& to) : ir_instruction(ir_instruction_tag::assign), from(from), to(to) {}
    std::vector<ir_value*> get_in_values() override { if (from.mode == ir_value_mode::var) return {&from}; else return {}; }
    std::vector<ir_variable*> get_out_variables() override { return {&to}; }
};

enum class ir_bin_op {
    add, sub, mul, div, rem, cmp
};

enum class ir_cmp_nan_mode {
    no_nan, unit, neg_unit
};

struct ir_bin_op_insruction : ir_instruction {
    ir_value first, second;
    ir_variable to;
    ir_bin_op op;
    ir_cmp_nan_mode nan_mode;

    ir_bin_op_insruction(const ir_value& first, const ir_value& second, const ir_variable& to, const ir_bin_op& op, const ir_cmp_nan_mode& nan_mode)
            : ir_instruction(ir_instruction_tag::bin_op), first(first), second(second), to(to), op(op), nan_mode(nan_mode) {}

    std::shared_ptr<ir_instruction> clone() const override {
        return std::make_shared<ir_bin_op_insruction>(first, second, to, op, nan_mode);
    }
    std::vector<ir_value*> get_in_values() override { return {&first, &second}; }
    std::vector<ir_variable*> get_out_variables() override { return {&to}; }
};

enum class ir_cmp_mode {
    eq, neq, lt, le, gt, ge
};

inline ir_cmp_mode negate_cmp_mode(const ir_cmp_mode& mode) {
    switch (mode) {
        case ir_cmp_mode::eq:  return ir_cmp_mode::neq;
        case ir_cmp_mode::neq: return ir_cmp_mode::eq;
        case ir_cmp_mode::lt:  return ir_cmp_mode::ge;
        case ir_cmp_mode::le:  return ir_cmp_mode::gt;
        case ir_cmp_mode::gt:  return ir_cmp_mode::le;
        case ir_cmp_mode::ge:  return ir_cmp_mode::lt;
        default_fail
    }
}

struct ir_cmp_jump_instruction : ir_instruction {
    ir_value first, second;
    ir_cmp_mode mode;
    ir_label label_true, label_false;
    ir_cmp_nan_mode nan_mode;

    ir_cmp_jump_instruction(const ir_value& first, const ir_value& second, ir_cmp_mode mode,
                            const ir_label& label_true, const ir_label& label_false, const ir_cmp_nan_mode& nan_mode)
            : ir_instruction(ir_instruction_tag::cmp_jump), first(first), second(second),
              mode(mode), label_true(label_true), label_false(label_false), nan_mode(nan_mode) {}

    std::shared_ptr<ir_instruction> clone() const override {
        return std::make_shared<ir_cmp_jump_instruction>(first, second, mode, label_true, label_false, nan_mode);
    }
    std::vector<ir_label> get_jump_labels() const override { return {label_true, label_false}; }
    std::vector<ir_value*> get_in_values() override { return {&first, &second}; }
};

enum class ir_convert_mode {
    i2l, l2i
};

struct ir_convert_instruction : ir_instruction {
    ir_value from;
    ir_variable to;
    ir_convert_mode mode;

    ir_convert_instruction(const ir_value& from, const ir_variable& to, ir_convert_mode mode)
            : ir_instruction(ir_instruction_tag::convert), from(from), to(to), mode(mode) {}

    std::shared_ptr<ir_instruction> clone() const override {
        return std::make_shared<ir_convert_instruction>(from, to, mode);
    }
    std::vector<ir_value*> get_in_values() override { return {&from}; }
    std::vector<ir_variable*> get_out_variables() override { return {&to}; }
};

struct ir_jump_instruction : ir_instruction {
    ir_label label;

    std::shared_ptr<ir_instruction> clone() const override {
        return std::make_shared<ir_jump_instruction>(label);
    }
    ir_jump_instruction(const ir_label& label) : ir_instruction(ir_instruction_tag::jump), label(label) {}
    std::vector<ir_label> get_jump_labels() const override { return {label}; }
};

struct ir_ret_instruction : ir_instruction {
    ir_value value;

    std::shared_ptr<ir_instruction> clone() const override {
        return std::make_shared<ir_ret_instruction>(value);
    }
    ir_ret_instruction(const ir_value& value) : ir_instruction(ir_instruction_tag::ret), value(value) {}
    bool exit_function() const override { return true; }
    std::vector<ir_value*> get_in_values() override { return {&value}; }
};

struct ir_phi_instruction : ir_instruction {
    std::vector<std::pair<ir_label, ir_value>> edges;
    ir_variable to;

    ir_phi_instruction(const std::vector<std::pair<ir_label, ir_value>>& edges, const ir_variable& to)
            : ir_instruction(ir_instruction_tag::phi), edges(edges), to(to) {}

    std::shared_ptr<ir_instruction> clone() const override {
        return std::make_shared<ir_phi_instruction>(edges, to);
    }
    std::vector<ir_value*> get_in_values() override {
        std::vector<ir_value*> result;
        std::transform(edges.begin(), edges.end(), std::back_inserter(result),
                       [](auto& item){ return &item.second; });
        return result;
    }
    std::vector<ir_variable*> get_out_variables() override { return {&to}; }
};

#endif //BADJVM_IR_INSTRUCTIONS_H
