/** -----------------------------------------------------------------------------
 * Copyright (c) 2023 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:     Lukas Schmid <lschmid@mit.edu>, Nathan Hughes <na26933@mit.edu>
 * AFFILIATION: MIT-SPARK Lab, Massachusetts Institute of Technology
 * YEAR:        2023
 * LICENSE:     BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include "config_utilities/factory.h"
#include "config_utilities/update.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/parsing/yaml.h"

namespace config {

namespace internal {

// Translate the parameter server to yaml code. This is inlined to shift the dependency on ROS to the downstream cpp
// file.
inline YAML::Node rosParamToYaml(const rclcpp::Parameter& param) {
  // NOTE(Oliver): Seems like there should be a clearer way to do this.
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return YAML::Node(param.as_bool());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return YAML::Node(param.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return YAML::Node(param.as_double());
    case rclcpp::ParameterType::PARAMETER_STRING:
      return YAML::Node(param.as_string());
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
      YAML::Node node(YAML::NodeType::Sequence);
      auto values = param.as_bool_array();
      for (bool v : values) {
        node.push_back(YAML::Node(v));
      }
      return node;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      YAML::Node node(YAML::NodeType::Sequence);
      auto values = param.as_integer_array();
      for (int v : values) {
        node.push_back(YAML::Node(v));
      }
      return node;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      YAML::Node node(YAML::NodeType::Sequence);
      auto values = param.as_double_array();
      for (double v : values) {
        node.push_back(YAML::Node(v));
      }
      return node;
    }
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
      YAML::Node node(YAML::NodeType::Sequence);
      auto values = param.as_string_array();
      for (std::string v : values) {
        node.push_back(YAML::Node(v));
      }
      return node;
    }
    default:
      return YAML::Node();
  }
}

inline YAML::Node rosToYaml(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface, 
        const std::string& name_space = "") {
  YAML::Node root;
  // ROS2 uses dots rather than slashes for parameter 'sub-namespaces' within nodes.
  std::string ros_ns = name_space;
  std::replace(ros_ns.begin(), ros_ns.end(), '/', '.');
  // Retrieve all parameters, with a namespace filter if specified
  auto filter = !ros_ns.empty() ? std::vector<std::string>{ros_ns} 
                      : std::vector<std::string>{};
  auto all_params = interface->list_parameters(filter, 
          rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);

  for (std::string& name : all_params.names) {
    // Get the parameter's value
    rclcpp::Parameter param;
    interface->get_parameter(name, param);

    std::replace(name.begin(), name.end(), '.', '/');
    std::vector<std::string> name_parts = splitNamespace(name, "/");
    std::string local_name = "";
    if (!name_parts.empty()) {
      local_name = name_parts.back();
      name_parts.pop_back();
    }

    const auto sub_namespace = joinNamespace(name_parts, "/");

    // Convert data to yaml.
    YAML::Node local_node;
    if (local_name.empty()) {
      local_node = rosParamToYaml(param);
    } else {
      local_node[local_name] = rosParamToYaml(param);
    }
    moveDownNamespace(local_node, sub_namespace, "/");
    mergeYamlNodes(root, local_node);
  }

  return root;
}

}  // namespace internal

/**
 * @brief Loads a config from a ROS2 node. This variant takes a parameter interface so 
 * can be used in the node's constructor.
 * 
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param interface Parameter interface of the ROS2 node holding the config parameters.
 * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 * Example: "my_config/my_sub_config".
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromRos(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface, 
        const std::string& name_space = "") {
  const YAML::Node yaml_node = internal::rosToYaml(interface, name_space);

  return fromYaml<ConfigT>(yaml_node, name_space);
}

/**
 * @brief Loads a config from a ROS2 node.
 *
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param node The ROS2 node holding the config parameters.
 * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '.'.
 * Example: "my_config/my_sub_config".
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromRos(rclcpp::Node::SharedPtr node, const std::string& name_space = "") {    
  return fromRos<ConfigT>(node->get_node_parameters_interface(), name_space);
}

/**
 * @brief Update the config with the current parameters in ROS2.
 * @note This function will update the field and check the validity of the config afterwards. 
 * If the config is invalid, the field will be reset to its original value.
 * @param config The config to update.
 * @param interface Parameter interface of the ROS2 node holding the config parameters.
 * @param name_space Optionally specify a name space to create the config from. 
 * Separate names with slashes '/'.
 */
template <typename ConfigT>
bool updateFromRos(ConfigT& config, 
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface, 
        const std::string& name_space = "") {
  const YAML::Node node = internal::rosToYaml(interface, name_space);
  return updateFields<ConfigT>(config, node, true, name_space);
}

/**
 * @brief Update the config with the current parameters in ROS2.
 * @note This function will update the field and check the validity of the config afterwards. 
 * If the config is invalid, the field will be reset to its original value.
 * @param config The config to update.
 * @param node The ROS2 node holding the config parameters.
 * @param name_space Optionally specify a name space to create the config from. 
 * Separate names with slashes '/'.
 */
template <typename ConfigT>
bool updateFromRos(ConfigT& config, const rclcpp::Node::SharedPtr node, 
        const std::string& name_space = "") {
  return updateFromRos<ConfigT>(config, node->get_node_parameters_interface(), name_space);
}

}  // namespace config
