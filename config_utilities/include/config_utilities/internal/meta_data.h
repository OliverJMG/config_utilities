#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace config::internal {

/**
 * @brief Struct that holds additional information about fields for printing.
 */
struct FieldInfo {
  // Name of the field. This is always given.
  std::string name;

  // Optional: Unit of the field.
  std::string unit;

  // The value of the field if the field is not a config.
  YAML::Node value;

  // Whether the field corresponds to its default value. Only queried if Settings().indicate_default_values is true.
  bool is_default = false;

  // If a field is a config this is the position of it's corresponding meta-data in the sub-configs vector.
  int subconfig_id = -1;
};

/**
 * @brief Meta-information struct that interfaces all the communication data when interacting with the configs. YAML is
 * used as internal data representation, so all conversions can be checked against yaml and all formatters, factories,
 * parsers can safely convert to yaml.
 */
struct MetaData {
  // Always get the name of the config if possible.
  std::string name;

  // Whether the data stored belongs to a virtual config.
  bool is_virtual_config = false;

  // Yaml node used to get or set the data of a config.
  YAML::Node data;

  // All additional field information queried for printing.
  std::vector<FieldInfo> field_infos;

  // All warnings issued by the validity checker or errors raised by the yaml parser.
  std::vector<std::string> errors;

  // If a config has sub-configs, they are stored here.
  std::vector<MetaData> sub_configs;

  // Utility to look up if there's any error messages in the data or its sub-configs.
  bool hasErrors() const;

  // Utility function so not every class needs to write their own recursion.
  void performOnAll(const std::function<void(MetaData&)>& func);
  void performOnAll(const std::function<void(const MetaData&)>& func) const;
};

}  // namespace config::internal
