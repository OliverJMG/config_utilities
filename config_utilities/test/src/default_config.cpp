
#include "config_utilities/test/default_config.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"

namespace config::test {

void declare_config(SubSubConfig& config) {
  config::name("SubSubConfig");
  config::field(config.i, "i");
  config::check(config.i, CheckMode::GT, 0, "i");
}

void declare_config(SubConfig& config) {
  config::name("SubConfig");
  config::field(config.i, "i");
  config::enter_namespace("nested_ns");
  config::field(config.sub_sub_config, "sub_sub_config");
  config::check(config.i, CheckMode::GT, 0, "i");
}

void declare_config(DefaultConfig& config) {
  config::name("DefaultConfig");
  config::field(config.i, "i", "m");
  config::field(config.f, "f", "s");
  config::field(config.d, "d", "m/s");
  config::field(config.b, "b");
  config::field(config.u8, "u8");
  config::field(config.s, "s");
  config::field(config.vec, "vec", "frames");
  config::field(config.map, "map");
  config::field(config.set, "set");
  config::field(config.mat, "mat");
  config::enum_field(config.my_enum, "my_enum", {"A", "B", "C"});
  config::enum_field(config.my_strange_enum,
                     "my_strange_enum",
                     {{DefaultConfig::StrangeEnum::kX, "X"},
                      {DefaultConfig::StrangeEnum::kY, "Y"},
                      {DefaultConfig::StrangeEnum::kZ, "Z"}});
  config::enter_namespace("sub_ns");
  config::field(config.sub_config, "sub_config", "sub_ns");
  config::switch_namespace("sub_sub_ns");
  config::field(config.sub_sub_config, "sub_sub_config", "sub_sub_ns");

  config::check(config.i, CheckMode::GT, 0, "i");
  config::check(config.f, CheckMode::GE, 0.f, "f");
  config::check(config.d, CheckMode::LT, 4.0, "d");
  config::check(config.u8, CheckMode::LE, uint8_t(5), "u8");
  config::check(config.s, CheckMode::EQ, std::string("test string"), "s");
  config::check(config.b, CheckMode::NE, false, "b");
  config::checkCondition(config.vec.size() == 3, "Param 'vec' must b of size '3'");
  config::checkInRange(config.d, 0.0, 500.0, "d");
}

YAML::Node DefaultConfig::defaultValues() {
  YAML::Node data;
  data["i"] = 1;
  data["f"] = 2.1f;
  data["d"] = 3.2;
  data["b"] = true;
  data["u8"] = 4;
  data["s"] = "test string";
  data["vec"] = std::vector<int>({1, 2, 3});
  const std::map<std::string, int> map({{"a", 1}, {"b", 2}, {"c", 3}});
  data["map"] = map;
  data["set"] = std::vector<float>({1.1f, 2.2, 3.3f});
  data["mat"].push_back(std::vector<double>({1, 0, 0}));
  data["mat"].push_back(std::vector<double>({0, 1, 0}));
  data["mat"].push_back(std::vector<double>({0, 0, 1}));
  data["my_enum"] = "A";
  data["my_strange_enum"] = "X";
  data["sub_ns"]["i"] = 1;
  data["sub_ns"]["nested_ns"]["i"] = 1;
  data["sub_sub_ns"]["i"] = 1;
  return data;
}

YAML::Node DefaultConfig::modifiedValues() {
  YAML::Node data;
  data["i"] = 2;
  data["f"] = -1.f;
  data["d"] = 3.1415926;
  data["b"] = false;
  data["u8"] = 255;
  data["s"] = "a different test string";
  data["vec"] = std::vector<int>({2, 3, 4, 5});
  const std::map<std::string, int> map({{"x", 24}, {"y", 25}, {"z", 26}});
  data["map"] = map;
  data["set"] = std::vector<float>({11.11, 22.22, 33.33, 44.44});
  data["mat"].push_back(std::vector<double>({1, 2, 3}));
  data["mat"].push_back(std::vector<double>({4, 5, 6}));
  data["mat"].push_back(std::vector<double>({7, 8, 9}));
  data["my_enum"] = "B";
  data["my_strange_enum"] = "Z";
  data["sub_ns"]["i"] = 2;
  data["sub_ns"]["nested_ns"]["i"] = 3;
  data["sub_sub_ns"]["i"] = 4;
  return data;
}

void DefaultConfig::expextDefaultValues() {
  EXPECT_EQ(i, 1);
  EXPECT_EQ(f, 2.1f);
  EXPECT_EQ(d, 3.2);
  EXPECT_EQ(b, true);
  EXPECT_EQ(u8, 4);
  EXPECT_EQ(s, "test string");
  EXPECT_EQ(vec, std::vector<int>({1, 2, 3}));
  const std::map<std::string, int> map({{"a", 1}, {"b", 2}, {"c", 3}});
  EXPECT_EQ(map, map);
  EXPECT_EQ(set, std::set<float>({1.1f, 2.2, 3.3f}));
  const auto mat = Eigen::Matrix<double, 3, 3>::Identity();
  EXPECT_EQ(mat, mat);
  EXPECT_EQ(my_enum, DefaultConfig::Enum::kA);
  EXPECT_EQ(my_strange_enum, DefaultConfig::StrangeEnum::kX);
  EXPECT_EQ(sub_config.i, 1);
  EXPECT_EQ(sub_config.sub_sub_config.i, 1);
  EXPECT_EQ(sub_sub_config.i, 1);
}

void DefaultConfig::expectModifiedValues() {
  EXPECT_EQ(i, 2);
  EXPECT_EQ(f, -1.f);
  EXPECT_EQ(d, 3.1415926);
  EXPECT_EQ(b, false);
  EXPECT_EQ(u8, 255);
  EXPECT_EQ(s, "a different test string");
  EXPECT_EQ(vec, std::vector<int>({2, 3, 4, 5}));
  const std::map<std::string, int> map({{"x", 24}, {"y", 25}, {"z", 26}});
  EXPECT_EQ(map, map);
  EXPECT_EQ(set, std::set<float>({11.11, 22.22, 33.33, 44.44}));
  Eigen::Matrix3d mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  EXPECT_EQ(mat, mat);
  EXPECT_EQ(my_enum, DefaultConfig::Enum::kB);
  EXPECT_EQ(my_strange_enum, DefaultConfig::StrangeEnum::kZ);
  EXPECT_EQ(sub_config.i, 2);
  EXPECT_EQ(sub_config.sub_sub_config.i, 3);
  EXPECT_EQ(sub_sub_config.i, 4);
}

}  // namespace config::test
