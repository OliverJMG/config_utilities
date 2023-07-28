#include "config_utilities/types/conversions.h"

#include <thread>

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/printing.h"

namespace config::test {

template <typename T>
std::string toYamlString(const T& conf) {
  const auto data = internal::Visitor::getValues(conf);
  YAML::Emitter out;
  out << data.data;
  std::string rep(out.c_str());
  return "\n" + rep + "\n";
}

struct ConversionStruct {
  int num_threads = 1;
  char some_character = 'a';
};

struct NoConversionStruct {
  int num_threads = 1;
  uint8_t some_character = 'a';
};

void declare_config(ConversionStruct& conf) {
  field<ThreadNumConversion>(conf.num_threads, "num_threads");
  field<CharConversion>(conf.some_character, "some_character");
}

void declare_config(NoConversionStruct& conf) {
  field(conf.num_threads, "num_threads");
  field(conf.some_character, "some_character");
}

// tests that we pull the right character from a string
TEST(Conversions, CharConversionCorrect) {
  std::string normal = "h";
  std::string too_long = "hello";
  std::string empty = "";

  std::string error_string = "";
  char value;
  CharConversion::fromIntermediate(normal, value, error_string);
  EXPECT_EQ(value, 'h');
  EXPECT_TRUE(error_string.empty());

  // Expect the field to be unchanged if the conversion fails.
  error_string = "";
  CharConversion::fromIntermediate(too_long, value, error_string);
  EXPECT_EQ(value, 'h');
  EXPECT_FALSE(error_string.empty());

  error_string = "";
  CharConversion::fromIntermediate(empty, value, error_string);
  EXPECT_EQ(value, 'h');
  EXPECT_FALSE(error_string.empty());
}

// tests that we can default autodetection of the number of cores
TEST(Conversions, ThreadNumConversionCorrect) {
  int auto_detect = -1;
  int manually_specified = 2;

  std::string error_string = "";
  int value = 0;
  ThreadNumConversion::fromIntermediate(auto_detect, value, error_string);
  EXPECT_GT(value, 0);

  value = 0;
  ThreadNumConversion::fromIntermediate(manually_specified, value, error_string);
  EXPECT_EQ(value, manually_specified);
}

// tests that conversions work as expected when parsing a config
TEST(Conversions, ConvertFields) {
  const std::string yaml_string = R"yaml(
num_threads: -1
some_character: c
)yaml";
  const auto node = YAML::Load(yaml_string);

  const auto conv = fromYaml<ConversionStruct>(node);
  EXPECT_GT(conv.num_threads, 0);
  EXPECT_EQ(conv.some_character, 'c');

  const auto no_conv = fromYaml<NoConversionStruct>(node);
  EXPECT_EQ(no_conv.num_threads, -1);
  EXPECT_EQ(no_conv.some_character, static_cast<uint16_t>('a'));

  // we don't know the hardware concurrency in advance, so we have to build this string
  std::stringstream conv_expected;
  conv_expected << std::endl;
  conv_expected << "num_threads: " << std::thread::hardware_concurrency() << std::endl;
  conv_expected << "some_character: c" << std::endl;
  EXPECT_EQ(conv_expected.str(), toYamlString(conv));

  std::string no_conv_expected = R"yaml(
num_threads: -1
some_character: 97
)yaml";
  EXPECT_EQ(no_conv_expected, toYamlString(no_conv));
}

// tests that conversions don't interfere when parsing a config where they don't apply
TEST(Conversions, ConvertFieldsPassthrough) {
  const std::string yaml_string = R"yaml(
num_threads: 5
some_character: 5
)yaml";
  const auto node = YAML::Load(yaml_string);

  const auto conv = fromYaml<ConversionStruct>(node);
  EXPECT_EQ(conv.num_threads, 5);
  EXPECT_EQ(conv.some_character, '5');

  const auto no_conv = fromYaml<NoConversionStruct>(node);
  EXPECT_EQ(no_conv.num_threads, 5);
  EXPECT_EQ(no_conv.some_character, 5);

  EXPECT_EQ(toYamlString(conv), yaml_string);
  EXPECT_EQ(toYamlString(no_conv), yaml_string);
}

}  // namespace config::test
