#include <gtest/gtest.h>

#include <arm_trajectory_commander/clock_fallback.hpp>

TEST(ClockFallback, UsesSteadyElapsedTimeUntilSimClockArrives)
{
  ClockFallback fallback(true, true);

  EXPECT_DOUBLE_EQ(fallback.effectiveNowS(0.0, 100.0), 0.0);
  EXPECT_TRUE(fallback.usingFallback());
  EXPECT_FALSE(fallback.hasRosClock());

  EXPECT_DOUBLE_EQ(fallback.effectiveNowS(0.0, 100.25), 0.25);
  EXPECT_TRUE(fallback.usingFallback());

  EXPECT_DOUBLE_EQ(fallback.effectiveNowS(3.0, 100.5), 3.0);
  EXPECT_FALSE(fallback.usingFallback());
  EXPECT_TRUE(fallback.hasRosClock());

  EXPECT_DOUBLE_EQ(fallback.effectiveNowS(0.0, 101.0), 0.0);
  EXPECT_FALSE(fallback.usingFallback());
}

TEST(ClockFallback, UsesRosTimeDirectlyForRealTimeConfigs)
{
  ClockFallback fallback(false, true);

  EXPECT_DOUBLE_EQ(fallback.effectiveNowS(0.0, 100.0), 0.0);
  EXPECT_FALSE(fallback.usingFallback());
  EXPECT_TRUE(fallback.hasRosClock());

  EXPECT_DOUBLE_EQ(fallback.effectiveNowS(2.5, 101.0), 2.5);
}

TEST(ClockFallback, CanRequireSimClock)
{
  ClockFallback fallback(true, false);

  EXPECT_DOUBLE_EQ(fallback.effectiveNowS(0.0, 100.0), 0.0);
  EXPECT_FALSE(fallback.usingFallback());
  EXPECT_FALSE(fallback.hasRosClock());
}

TEST(ClockFallback, ExplicitSimClockTakesPriorityOverFallback)
{
  ClockFallback fallback(true, true);

  EXPECT_DOUBLE_EQ(fallback.effectiveNowS(0.0, 50.0), 0.0);
  EXPECT_TRUE(fallback.usingFallback());
  EXPECT_FALSE(fallback.hasRosClock());

  EXPECT_DOUBLE_EQ(fallback.effectiveNowS(12.5, 51.0), 12.5);
  EXPECT_FALSE(fallback.usingFallback());
  EXPECT_TRUE(fallback.hasRosClock());
}
