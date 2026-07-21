#include <policy_inference/recurrent_state.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace policy_inference
{
namespace
{

TEST(RecurrentState, ConfiguresAppendsUpdatesAndResets)
{
  RecurrentState state;
  state.configure({1, 1, 4});
  ASSERT_TRUE(state.enabled());
  ASSERT_EQ(state.size(), 4U);
  ASSERT_EQ(state.shape(), (std::vector<int64_t>{1, 1, 4}));

  TensorMap inputs;
  state.appendInput(inputs);
  ASSERT_EQ(inputs.at("h_in"), Tensor(4, 0.0F));

  state.updateFromOutput({{"h_out", {1.0F, 2.0F, 3.0F, 4.0F}}});
  EXPECT_EQ(state.values(), (Tensor{1.0F, 2.0F, 3.0F, 4.0F}));

  state.reset();
  EXPECT_EQ(state.values(), Tensor(4, 0.0F));
}

TEST(RecurrentState, RejectsMissingOrWrongSizedOutput)
{
  RecurrentState state;
  state.configure({1, 2});
  EXPECT_THROW(state.updateFromOutput({}), std::invalid_argument);
  EXPECT_THROW(state.updateFromOutput({{"h_out", {1.0F}}}), std::invalid_argument);
  EXPECT_THROW(
    state.updateFromOutput(
      {{"h_out", {1.0F, std::numeric_limits<float>::quiet_NaN()}}}),
    std::invalid_argument);
  EXPECT_THROW(
    state.updateFromOutput({{"h_out", {1.0F, 2.0F}}}, ""),
    std::invalid_argument);
}

TEST(RecurrentState, RejectsUnknownOrImpossibleShape)
{
  RecurrentState state;
  EXPECT_THROW(state.configure({}), std::invalid_argument);
  EXPECT_THROW(state.configure({1, -1, 4}), std::invalid_argument);
  EXPECT_THROW(state.configure({1, 0, 4}), std::invalid_argument);
}

TEST(RecurrentState, DisabledStateDoesNotMutateInputs)
{
  RecurrentState state;
  TensorMap inputs;
  state.appendInput(inputs);
  state.updateFromOutput({});
  EXPECT_TRUE(inputs.empty());
  EXPECT_FALSE(state.enabled());

  state.configure({1, 2});
  state.disable();
  EXPECT_TRUE(state.shape().empty());
  EXPECT_TRUE(state.values().empty());
}

}  // namespace
}  // namespace policy_inference
