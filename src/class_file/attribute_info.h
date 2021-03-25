#ifndef BADJVM_ATTRIBUTE_INFO_H
#define BADJVM_ATTRIBUTE_INFO_H

#include "cp_info.h"
#include "class_file.h"
#include <cstdint>
#include <fstream>
#include <vector>

struct attribute_info {
    uint16_t attribute_name_index;
    uint32_t attribute_length;
    uint8_t* info;

    static attribute_info read(std::ifstream& stream, const struct class_file* a_class);
};

static constexpr const char* ai_constant_value = "ConstantValue";
static constexpr const char* ai_code = "Code";
static constexpr const char* ai_stack_map_table = "StackMapTable";
static constexpr const char* ai_exceptions = "Exceptions";
static constexpr const char* ai_inner_classes = "InnerClasses";
static constexpr const char* ai_enclosing_method = "EnclosingMethod";
static constexpr const char* ai_synthetic = "Synthetic";
static constexpr const char* ai_signature = "Signature";
static constexpr const char* ai_source_file = "SourceFile";
static constexpr const char* ai_source_debug_extension = "SourceDebugExtension";
static constexpr const char* ai_line_number_table = "LineNumberTable";
static constexpr const char* ai_local_variable_table = "LocalVariableTable";
static constexpr const char* ai_local_variable_type_table = "LocalVariableTypeTable";
static constexpr const char* ai_deprecated = "Deprecated";
static constexpr const char* ai_runtime_visible_annotations = "RuntimeVisibleAnnotations";
static constexpr const char* ai_runtime_invisible_annotations = "RuntimeInvisibleAnnotations";
static constexpr const char* ai_runtime_visible_parameter_annotations = "RuntimeVisibleParameterAnnotations";
static constexpr const char* ai_runtime_invisible_parameter_annotations = "RuntimeInvisibleParameterAnnotations";
static constexpr const char* ai_annotation_default = "AnnotationDefault";
static constexpr const char* ai_bootstrap_methods = "BootstrapMethods";

#endif //BADJVM_ATTRIBUTE_INFO_H
