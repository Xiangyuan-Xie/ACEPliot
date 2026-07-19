#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#ifndef ACEPILOT_CONTROL_MODEL_HAS_URDFDOM
#define ACEPILOT_CONTROL_MODEL_HAS_URDFDOM 0
#endif

#if ACEPILOT_CONTROL_MODEL_HAS_URDFDOM
#include <urdf_parser/urdf_parser.h>
#endif

#ifndef TEST
#define TEST(test_suite_name, test_name) void test_suite_name ## _ ## test_name()
#endif

namespace
{

namespace pt = boost::property_tree;

std::filesystem::path controlModelPath()
{
#ifdef ACEPILOT_CONTROL_MODEL_PATH
  return ACEPILOT_CONTROL_MODEL_PATH;
#else
  std::filesystem::path source_path = __FILE__;
  if (source_path.is_relative()) {
    source_path = std::filesystem::current_path() / source_path;
  }
  return source_path.parent_path().parent_path() /
         "models/x500_arm2x_control.urdf";
#endif
}

std::string attribute(const pt::ptree & node, const std::string & name)
{
  return node.get<std::string>("<xmlattr>." + name);
}

using NamedNodes = std::map<std::string, const pt::ptree *>;

NamedNodes namedChildren(const pt::ptree & parent, const std::string & tag)
{
  NamedNodes result;
  for (const auto & child : parent) {
    if (child.first != tag) {
      continue;
    }
    const std::string name = attribute(child.second, "name");
    if (!result.emplace(name, &child.second).second) {
      throw std::runtime_error("Duplicate <" + tag + "> named " + name);
    }
  }
  return result;
}

std::vector<std::string> childNames(const pt::ptree & parent, const std::string & tag)
{
  std::vector<std::string> names;
  for (const auto & child : parent) {
    if (child.first == tag) {
      names.push_back(attribute(child.second, "name"));
    }
  }
  return names;
}

struct LoadedControlModel
{
  std::string xml;
  pt::ptree document;
};

LoadedControlModel loadControlModel()
{
  const std::filesystem::path model_path = controlModelPath();
  std::ifstream input(model_path, std::ios::binary);
  if (!input.is_open()) {
    throw std::runtime_error("Unable to open " + model_path.string());
  }

  std::ostringstream buffer;
  buffer << input.rdbuf();

  LoadedControlModel model;
  model.xml = buffer.str();
  std::istringstream xml_stream(model.xml);
  pt::read_xml(xml_stream, model.document, pt::xml_parser::trim_whitespace);
  model.document.get_child("robot");
  return model;
}

TEST(ControlModel, ParsesAsUrdfAndContainsOnlyTheControlTree)
{
  const LoadedControlModel loaded = loadControlModel();
  const pt::ptree & robot = loaded.document.get_child("robot");
  EXPECT_EQ(attribute(robot, "name"), "x500_arm2x_control");

  const std::vector<std::string> expected_links = {
    "base_link", "link_1", "link_2", "link_3", "link_4", "ee_link"};
  const std::vector<std::string> expected_joints = {
    "joint_1", "joint_2", "joint_3", "joint_4", "ee_fixed_joint"};
  EXPECT_EQ(childNames(robot, "link"), expected_links);
  EXPECT_EQ(childNames(robot, "joint"), expected_joints);
  EXPECT_NE(
    loaded.xml.find(
      "Source URDF SHA256: "
      "b8f46a3c5216b39dfa51f4560701b538045eed3552e38f578a0b2194e59bc918"),
    std::string::npos);

  const NamedNodes links = namedChildren(robot, "link");
  for (const auto & named_link : links) {
    EXPECT_EQ(named_link.second->count("visual"), 0U) << named_link.first;
    EXPECT_EQ(named_link.second->count("collision"), 0U) << named_link.first;
  }
  EXPECT_EQ(loaded.xml.find("<mesh"), std::string::npos);

  for (const char * omitted : {
      "link_5", "gripper_left", "gripper_right", "rotor_1", "rotor_2", "rotor_3",
      "rotor_4"})
  {
    EXPECT_EQ(links.count(omitted), 0U) << omitted;
  }

#if ACEPILOT_CONTROL_MODEL_HAS_URDFDOM
  const urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(loaded.xml);
  ASSERT_TRUE(urdf_model);
  ASSERT_TRUE(urdf_model->getRoot());
  EXPECT_EQ(urdf_model->getName(), "x500_arm2x_control");
  EXPECT_EQ(urdf_model->getRoot()->name, "base_link");
  EXPECT_EQ(urdf_model->links_.size(), expected_links.size());
  EXPECT_EQ(urdf_model->joints_.size(), expected_joints.size());
#endif
}

TEST(ControlModel, PreservesAndLumpsSourceInertialsExactly)
{
  struct ExpectedInertial
  {
    const char * link;
    const char * origin;
    const char * mass;
    std::array<const char *, 6> inertia;
  };

  const std::vector<ExpectedInertial> expected = {
    {"base_link", "-0.013121 0.00017397 -0.067418", "1.8403",
      {"0.019333", "5.072E-05", "0.0010586", "0.02214", "-4.352E-05", "0.028275"}},
    {"link_1", "-0.025459 0.17263 -0.00094115", "0.15156",
      {"0.00087586", "0.0001761", "-2.32E-06", "9.027E-05", "7.76E-06",
        "0.00092044"}},
    {"link_2", "-0.039172 -0.18259 -0.00068453", "0.14766",
      {"0.0016083", "-4.92E-05", "-1.8E-07", "4.02E-05", "-8.08E-06",
        "0.0016046"}},
    {"link_3", "0.0055284 -0.033186 -3.76E-06", "0.0636",
      {"2.011E-05", "1.71E-06", "0", "1.802E-05", "0", "2.34E-05"}},
    {"link_4", "-8.08936229013e-05 -0.0013581756244 0.043125786869", "0.15272",
      {"0.000114699427694", "1.57880455352e-06", "1.62506217408e-07",
        "0.000340738958485", "-4.73876972285e-06", "0.000267101289269"}},
  };
  const std::array<const char *, 6> inertia_attributes = {
    "ixx", "ixy", "ixz", "iyy", "iyz", "izz"};

  const LoadedControlModel loaded = loadControlModel();
  const pt::ptree & robot = loaded.document.get_child("robot");
  const NamedNodes links = namedChildren(robot, "link");
  for (const ExpectedInertial & item : expected) {
    ASSERT_EQ(links.count(item.link), 1U) << item.link;
    const pt::ptree & inertial = links.at(item.link)->get_child("inertial");
    EXPECT_EQ(attribute(inertial.get_child("origin"), "xyz"), item.origin) << item.link;
    EXPECT_EQ(attribute(inertial.get_child("origin"), "rpy"), "0 0 0") << item.link;
    EXPECT_EQ(attribute(inertial.get_child("mass"), "value"), item.mass) << item.link;

    const pt::ptree & tensor = inertial.get_child("inertia");
    for (std::size_t index = 0; index < inertia_attributes.size(); ++index) {
      EXPECT_EQ(attribute(tensor, inertia_attributes[index]), item.inertia[index])
        << item.link << " " << inertia_attributes[index];
    }
  }

  ASSERT_EQ(links.count("ee_link"), 1U);
  EXPECT_EQ(links.at("ee_link")->count("inertial"), 0U);
}

TEST(ControlModel, PreservesExactJointChainTransformsAndLimits)
{
  struct ExpectedJoint
  {
    const char * name;
    const char * parent;
    const char * child;
    const char * xyz;
    const char * rpy;
    const char * lower;
    const char * upper;
    const char * effort;
    const char * velocity;
  };

  const std::vector<ExpectedJoint> expected = {
    {"joint_1", "base_link", "link_1", "0 0.0002 -0.1503", "-1.5708 0 3.1416",
      "-2.6485", "2.6485", "5.24", "6.28"},
    {"joint_2", "link_1", "link_2", "-0.04123 0.2314 -0.0005", "0 0 3.1416",
      "0", "3.1415", "3.92", "7.85"},
    {"joint_3", "link_2", "link_3", "-0.04098 -0.2686 0", "0 0 0",
      "-2.6485", "2.6485", "2.75", "4.71"},
    {"joint_4", "link_3", "link_4", "0 -0.05189 0", "1.5708 1.5708 0",
      "-3.1415", "3.1415", "1.11", "10.47"},
  };

  const LoadedControlModel loaded = loadControlModel();
  const pt::ptree & robot = loaded.document.get_child("robot");
  const NamedNodes joints = namedChildren(robot, "joint");
  for (const ExpectedJoint & item : expected) {
    ASSERT_EQ(joints.count(item.name), 1U) << item.name;
    const pt::ptree & joint = *joints.at(item.name);
    EXPECT_EQ(attribute(joint, "type"), "revolute") << item.name;
    EXPECT_EQ(attribute(joint.get_child("parent"), "link"), item.parent) << item.name;
    EXPECT_EQ(attribute(joint.get_child("child"), "link"), item.child) << item.name;
    EXPECT_EQ(attribute(joint.get_child("origin"), "xyz"), item.xyz) << item.name;
    EXPECT_EQ(attribute(joint.get_child("origin"), "rpy"), item.rpy) << item.name;
    EXPECT_EQ(attribute(joint.get_child("axis"), "xyz"), "0 0 1") << item.name;

    const pt::ptree & limit = joint.get_child("limit");
    EXPECT_EQ(attribute(limit, "lower"), item.lower) << item.name;
    EXPECT_EQ(attribute(limit, "upper"), item.upper) << item.name;
    EXPECT_EQ(attribute(limit, "effort"), item.effort) << item.name;
    EXPECT_EQ(attribute(limit, "velocity"), item.velocity) << item.name;
  }

  ASSERT_EQ(joints.count("ee_fixed_joint"), 1U);
  const pt::ptree & ee_joint = *joints.at("ee_fixed_joint");
  EXPECT_EQ(attribute(ee_joint, "type"), "fixed");
  EXPECT_EQ(attribute(ee_joint.get_child("parent"), "link"), "link_4");
  EXPECT_EQ(attribute(ee_joint.get_child("child"), "link"), "ee_link");
  EXPECT_EQ(attribute(ee_joint.get_child("origin"), "xyz"), "0 0.008255 0.0643");
  EXPECT_EQ(attribute(ee_joint.get_child("origin"), "rpy"), "0 0 0");
  EXPECT_EQ(ee_joint.count("axis"), 0U);
  EXPECT_EQ(ee_joint.count("limit"), 0U);

  EXPECT_EQ(joints.count("joint_5"), 0U);
  EXPECT_EQ(joints.count("joint_gripper_left"), 0U);
  EXPECT_EQ(joints.count("joint_gripper_right"), 0U);
}

}  // namespace
