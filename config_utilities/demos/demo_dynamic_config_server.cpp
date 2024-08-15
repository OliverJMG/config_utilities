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

/**
 * This demo shows how to use config_utilities with ROS.
 */

#include <iostream>
#include <string>

#include <ros/ros.h>

#include "config_utilities/config.h"                 // Enables declare_config().
#include "config_utilities/dynamic_config.h"         // Enables DynamicConfig and DynamicConfigServer.
#include "config_utilities/logging/log_to_stdout.h"  // Log config_utilities messages.
#include "config_utilities/parsing/ros.h"            // Enable fromRos() and the RosDynamicConfigServer.
#include "config_utilities/printing.h"               // Enable toString()
#include "config_utilities/types/eigen_matrix.h"     // Enable parsing and printing of Eigen::Matrix types.
#include "config_utilities/types/enum.h"             // Enable parsing and printing of enum types.

namespace demo {

// A sub-struct for later use.
struct SubConfig {
  float f = 1.1f;
  std::string s = "test";
};

// A struct that represents what we want to be a config.
// Requirements for a config struct: is default constructable.
struct MyConfig {
  int i = 100;
  double distance = 42;
  bool b = true;
  std::vector<int> vec = {1, 2, 3};
  std::map<std::string, int> map = {{"a", 1}, {"b", 2}, {"c", 3}};
  Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Identity();
  enum class MyEnum { kA, kB, kC } my_enum = MyEnum::kA;
  SubConfig sub_config;
};

// Defining 'void declare_config(T& config)' function labels a struct as config.
// It **MUST** be declared beforehand if being used in another declare_config
void declare_config(SubConfig&);

// All config properties are specified within declare_config.
void declare_config(MyConfig& config) {
  config::name("MyConfig");
  config::field(config.i, "i");
  config::field(config.distance, "distance", "m");
  config::field(config.b, "b");
  config::field(config.vec, "vec");
  config::field(config.map, "map");
  config::field(config.mat, "mat");
  config::enum_field(config.my_enum, "my_enum", {"A", "B", "C"});
  config::field(config.sub_config, "sub_config");

  config::check(config.i, config::CheckMode::GT, 0, "i");
  config::checkInRange(config.distance, 0.0, 100.0, "distance");
}

// Declaration of the subconfig.
void declare_config(SubConfig& config) {
  using namespace config;
  name("SubConfig");
  field(config.f, "f");
  field(config.s, "s");
  checkIsOneOf(config.f, {0.0f, 1.1f, 2.2f, 3.3f}, "f");
}

// Declare an object with a dynamic config.
template <typename ConfigT>
class DynamicConfigObject {
 public:
  explicit DynamicConfigObject(const std::string& name, const ConfigT& initial_config)
      : name_(name), config_(name, initial_config) {
    // The above initialization registers the dynamic config with its global identifier name, which we resolve using the
    // nodehandle to get a unique name for every node/object. The config is initialized with the current ROS parameters.
    // The callback is called whenever the config is updated.
    config_.setCallback(std::bind(&DynamicConfigObject::callback, this));
  }

 private:
  const std::string name_;
  config::DynamicConfig<ConfigT> config_;

  void callback() const {
    // Do something with the new config.
    std::cout << "Received new config for " << name_ << ":\n" << config::toString(config_.get()) << std::endl;
  }
};

}  // namespace demo

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_config_server");
  ros::NodeHandle nh("~");

  // Advertize setting and getting dynamic configs via ros topics.
  config::RosDynamicConfigServer server(nh);

  // Create dynamic config objects. These will automatically register their config with the server.
  demo::DynamicConfigObject obj("dynamic_object_config", demo::MyConfig());

  // Initialize another config with different name and params.
  nh.setParam("i", 42);
  nh.setParam("distance", 42.0);
  nh.setParam("b", false);
  nh.setParam("vec", std::vector<int>());
  nh.setParam("map", std::map<std::string, int>({{"ASD", 42}}));
  demo::DynamicConfigObject obj2("another_config", config::fromRos<demo::MyConfig>(nh));

  // Initialize a subconfig.
  demo::DynamicConfigObject sub_obj("sub_config", demo::SubConfig());

  // Spin to keep the node alive.
  ros::spin();
  return 0;
}
