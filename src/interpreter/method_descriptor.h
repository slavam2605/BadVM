#ifndef BADJVM_METHOD_DESCRIPTOR_H
#define BADJVM_METHOD_DESCRIPTOR_H

#include <vector>
#include <string>

enum class base_type_descriptor {
    reference_d, byte_d, char_d, double_d, float_d, int_d, long_d, short_d, boolean_d
};

struct field_descriptor {
    char base_type = '\0';
    std::string object_type = "";
    int array_dimensions = 0;
    field_descriptor* array_type = nullptr;

    base_type_descriptor get_base_type() const;
    uint8_t get_field_size() const;
    bool is_int_value() const;

    static field_descriptor parse(const std::string& source);
    static field_descriptor parse(const std::string& source, int& pos);
};

struct method_descriptor {
    std::vector<field_descriptor> parameters;
    field_descriptor return_descriptor;
    bool return_void = false;

    static method_descriptor parse(const std::string& source);
};


#endif //BADJVM_METHOD_DESCRIPTOR_H
