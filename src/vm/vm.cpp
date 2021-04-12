#include <iostream>
#include <iomanip>
#include "vm.h"
#include "../utils/constant_pool_utils.h"
#include "../utils/utils.h"
#include "../class_file/access_flags.h"
#include <chrono>
#include <cmath>

using namespace std;

// TODO specific to Java 8
void load_system_class(vm* jvm) {
    auto system_class = jvm->get_or_load_class("java/lang/System");
    auto [target_class, code_info] = jvm->get_code_info(system_class, "initializeSystemClass", "()V");
    jvm->interpret(target_class, code_info, vector<jvm_value>());
}

void vm::start(const std::string& class_name) {
    // TODO make JVM initialization consistent with specification
//    load_system_class(this);

    auto main_class = get_or_load_class(class_name);
    for (const auto& method : main_class->methods) {
        if ((method.access_flags & ACC_STATIC) == 0)
            continue;

        auto method_name = read_utf8_string(main_class, method.name_index);
        if (method_name == /*TODO "main"*/"dfoo") {
            vector<jvm_value> main_arguments;
//            main_arguments.push_back(jvm_value::as_reference(0)); TODO bin_op null for main method
            compile_and_invoke(main_class, method, move(main_arguments));
//            cout << interpret(main_class, info, move(main_arguments)).int_value << endl;
            return;
        }
    }
    cerr << "Failed to find a static method main(String[])" << endl;
}

class_file* vm::get_or_load_class(const string& name) {
    if (loaded_classes.find(name) == loaded_classes.end()) {
        class_file* loaded_class = class_file::read(this, name);
        loaded_classes[name] = loaded_class;

        // find and compile_and_invoke <clinit> for a new class
        for (const auto& method : loaded_class->methods) {
            auto method_name = read_utf8_string(loaded_class, method.name_index);
            if (method_name != "<clinit>")
                continue;

            for (const auto& attribute : method.attributes) {
                const auto& attribute_name = read_utf8_string(loaded_class, attribute.attribute_name_index);
                if (attribute_name != ai_code)
                    continue;

                const auto* code_info = reinterpret_cast<code_attribute_info*>(attribute.info);
                interpret(loaded_class, code_info, vector<jvm_value>());
            }
        }
    }
    return loaded_classes[name];
}

class_file* vm::get_or_load_class(const class_file* current_class, const cp_class_info* class_info) {
    auto class_name = read_utf8_string(current_class, class_info->name_index);
    return get_or_load_class(class_name);
}

vector<class_file*> vm::load_super_classes_and_self(class_file* starting_class) {
    vector<class_file*> result;
    class_file* current = starting_class;
    while (current != nullptr) {
        result.push_back(current);
        if (current->super_class == 0)
            break;

        current = get_or_load_class(current, read_class_info(current, current->super_class));
    }
    return result;
}

pair<const class_file*, const code_attribute_info*> vm::get_code_info(const class_file* current_class, const cp_fmi_ref_info* method_ref) {
    const auto* class_info = read_class_info(current_class, method_ref->class_index);
    const auto& target_method_class = get_or_load_class(current_class, class_info);
    const auto* name_and_type = read_name_and_type(current_class, method_ref->name_and_type_index);
    const auto& target_method_name = read_utf8_string(current_class, name_and_type->name_index);
    const auto& target_method_descriptor = read_utf8_string(current_class, name_and_type->descriptor_index);

    return get_code_info(target_method_class, target_method_name, target_method_descriptor);
}

pair<const class_file*, const code_attribute_info*> vm::get_code_info(class_file* target_class, const string& target_method_name, const string& target_method_descriptor) {
    auto super_classes = load_super_classes_and_self(target_class);
    for (auto some_class : super_classes) {
        for (const auto& method : some_class->methods) {
            const auto& method_name = read_utf8_string(some_class, method.name_index);
            if (method_name != target_method_name)
                continue;

            const auto& method_descriptor = read_utf8_string(some_class, method.descriptor_index);
            if (method_descriptor != target_method_descriptor)
                continue;

            for (const auto& attribute : method.attributes) {
                const auto& attribute_name = read_utf8_string(some_class, attribute.attribute_name_index);
                if (attribute_name != ai_code)
                    continue;

                const auto* code_info = reinterpret_cast<code_attribute_info*>(attribute.info);
                return make_pair(some_class, code_info);
            }
            // TODO implement natives
//            cerr << "Failed to find a Code attribute for a method " << some_class->class_name << "." << method_name << ": " << method_descriptor << endl;
            return make_pair(some_class, nullptr);
//            throw runtime_error("Failed to find a Code attribute for a method " + method_name + ": " + method_descriptor);
        }
    }

    throw runtime_error("Failed to find a method " + target_method_name + ": " + target_method_descriptor);
}

bool vm::is_java_lang_object_init(const class_file* current_class, const cp_fmi_ref_info* method_info) {
    const auto class_info = read_class_info(current_class, method_info->class_index);
    const auto& class_name = read_utf8_string(current_class, class_info->name_index);
    if (class_name != "java/lang/Object")
        return false;

    const auto name_and_type = read_name_and_type(current_class, method_info->name_and_type_index);
    const auto& method_name = read_utf8_string(current_class, name_and_type->name_index);
    return method_name == "<init>";
}

ifstream vm::open_class_file(const string& class_name) {
    static vector<string> class_path {"", "rt/"};

    for (const auto& path : class_path) {
        ifstream result(path + class_name + ".class", ios::binary);
        if (!result.is_open())
            continue;

//        cout << "Loading class " << class_name << endl;
        return result;
    }
    throw runtime_error("Failed to load class " + class_name);
}

const code_attribute_info* vm::get_code_info(const class_file* current_class, const method_info& method) {
    for (const auto& attribute : method.attributes) {
        auto attribute_name = read_utf8_string(current_class, attribute.attribute_name_index);
        if (attribute_name != ai_code)
            continue;

        return reinterpret_cast<code_attribute_info*>(attribute.info);
    }
    throw runtime_error("Failed to find Code attribute");
}

template <class T>
void benchmark(T (*foo)(), int warmup_count = 3, int measure_count = 7) {
    T result;
    vector<double> measures;

    for (int i = 0; i < warmup_count + measure_count; i++) {
        auto start = chrono::steady_clock::now();
        result = foo();
        auto end = chrono::steady_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
        cout << "Measure #" << i << ": " << duration.count() << " ms";
        if (i < warmup_count) {
            cout << " (warmup)";
        } else {
            measures.push_back(duration.count());
        }
        cout << endl;
    }
    double sum = 0.0;
    double sum_sq = 0.0;
    for (int i = 0; i < measures.size(); i++) {
        sum += measures[i];
        sum_sq += measures[i] * measures[i];
    }
    double average = sum / measures.size();
    double mean_sq_var = sqrt(sum_sq / measures.size() - average * average);

    cout << result << endl;
    cout << "Time: " << average << " ms +- " << mean_sq_var << " ms" << endl;
}

void vm::compile_and_invoke(const class_file* current_class, const method_info& method, const vector<jvm_value>&& parameters) {
    if (false) {
        cout << interpret(current_class, method, move(parameters)).int_value << endl;
    } else {
        auto code_info = get_code_info(current_class, method);
        auto compiled_fun = compiler.compile(current_class, method, code_info);
        auto fun_ptr = reinterpret_cast<double (*)()>(const_cast<void*>(compiled_fun));

//        benchmark(fun_ptr);
        cout << setprecision(20) << fun_ptr() << endl;
    }
}




