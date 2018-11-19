#include "gtest\gtest.h"
//#include "unittest_solver.cpp"
#include "../NelderMeadSolver/SimplexFactory.h"
#include "../NelderMeadSolver/Solver.h"
#include "../KinematicBase/ValueWrapperJoint.h"
#include "../KinematicBase/RotaryJoint.h"
#include "../KinematicBase/LinearJoint.h"
#include "../KinematicBase/Chain.h"

#include <array>


using namespace kinematics;
using namespace nmsolver;

using PointAndTwoDoubles = std::tuple<kinematics::Point, double, double>;

class TwoJointsChain : public ::testing::TestWithParam<PointAndTwoDoubles> {
protected:

  void SetUp() override {
    DHConvention c(5.0, 0.0, 0.0, 0.0);

    j1 = std::make_shared<RotaryJoint>(c);
    j2 = std::make_shared<RotaryJoint>(c);

    chain.addJoint(j1);
    chain.addJoint(j2);
  }

  std::shared_ptr<RotaryJoint> j1;
  std::shared_ptr<RotaryJoint> j2;
  Chain chain;

  const double cmp_point = 1e-3;
};

class ThreeJointsChain : public ::testing::TestWithParam<kinematics::Point> {
protected:

  void SetUp() override {
    DHConvention c(5.0, 0.0, 0.0, 0.0);

    auto j1 = std::make_shared<RotaryJoint>(c);
    auto j2 = std::make_shared<RotaryJoint>(c);
    auto j3 = std::make_shared<RotaryJoint>(c);

    chain.addJoint(j1);
    chain.addJoint(j2);
    chain.addJoint(j3);
  }

  Chain chain;

  const double cmp_point = 1e-3;
};

class ThreeJointsChainZ : public ThreeJointsChain {};

class TenJointsChain : public ::testing::TestWithParam<kinematics::Point> {
protected:

  void SetUp() override{
    DHConvention c1(5.0, 0.0, 0.0, 0.0);
    DHConvention c2(0.0, - pi_2, 0.0, 0.0);

    auto joint_to_chain = [this](DHConvention& c) {
      auto joint1 = std::make_shared<RotaryJoint>(c);
      joint1->setMinMax(-pi, pi);
      chain.addJoint(joint1);
    };

    for (int i = 0; i < 5; ++i)
    {
      joint_to_chain(c1);
      joint_to_chain(c2);
    }
  }

  Chain chain;

  const double cmp_point = 1e-3;
};

TEST_P(TwoJointsChain, Standard)
{
  j1->setMinMax(-pi, pi);
  j2->setMinMax(-pi, pi);

  auto params = GetParam();
  auto point = std::get<0>(params);
  auto expec1 = std::get<1>(params);
  auto expec2 = std::get<2>(params);
  auto success = chain.goInverse(point);
  auto after = chain.goForward();

  EXPECT_TRUE(success);
  ASSERT_NEAR(point.x(), after.x(), cmp_point);
  ASSERT_NEAR(point.y(), after.y(), cmp_point);
  ASSERT_NEAR(point.z(), after.z(), cmp_point);
  ASSERT_NEAR(expec1, j1->getJointValue(), 0.1);
  ASSERT_NEAR(expec2, j2->getJointValue(), 0.1);
}

TEST_P(TwoJointsChain, NoLimits)
{
  auto params = GetParam();
  auto point = std::get<0>(params);
  auto expec1 = std::get<1>(params);
  auto expec2 = std::get<2>(params);
  auto success = chain.goInverse(point);
  auto after = chain.goForward();

  EXPECT_TRUE(success);
  ASSERT_NEAR(point.x(), after.x(), cmp_point);
  ASSERT_NEAR(point.y(), after.y(), cmp_point);
  ASSERT_NEAR(point.z(), after.z(), cmp_point);
  ASSERT_NEAR(expec1, j1->getJointValue(), 0.1);
  ASSERT_NEAR(expec2, j2->getJointValue(), 0.1);
}

INSTANTIATE_TEST_CASE_P(Test, TwoJointsChain,
  ::testing::Values(
    PointAndTwoDoubles{ kinematics::Point(0.0, 5.0, 0.0), 0.523602, 2.09439 },
    PointAndTwoDoubles{ kinematics::Point(0.0, 3.0, 0.0), 2.83689999957439, -2.53221 }
));

TEST_P(ThreeJointsChain, Standard)
{
  auto point = GetParam();
  auto success = chain.goInverse(point);
  auto after = chain.goForward();

  EXPECT_TRUE(success);
  ASSERT_NEAR(point.x(), after.x(), cmp_point);
  ASSERT_NEAR(point.y(), after.y(), cmp_point);
  ASSERT_NEAR(point.z(), after.z(), cmp_point);
}

INSTANTIATE_TEST_CASE_P(Test, ThreeJointsChain,
  ::testing::Values(
    kinematics::Point(15.0, 0.0, 0.0),
    kinematics::Point(5.0, 0.0, 0.0),
    kinematics::Point(3.0, 0.0, 0.0),
    kinematics::Point(2.0, 0.0, 0.0),
    kinematics::Point(0.0, 15.0, 0.0),
    kinematics::Point(0.0, 5.0, 0.0),
    kinematics::Point(0.0, 3.0, 0.0),
    kinematics::Point(0.0, 2.0, 0.0)));

TEST_P(ThreeJointsChainZ, UnreachableZ)
{
  auto point = GetParam();
  auto success = chain.goInverse(point);
  auto after = chain.goForward();

  EXPECT_FALSE(success);
  ASSERT_NEAR(point.x(), after.x(), cmp_point);
  ASSERT_NEAR(point.y(), after.y(), cmp_point);
  ASSERT_GT(point.z(), after.z());
}

INSTANTIATE_TEST_CASE_P(Test, ThreeJointsChainZ,
  ::testing::Values(
    kinematics::Point(0.0, 0.0, 1.0),
    kinematics::Point(0.0, 0.0, 5.0),
    kinematics::Point(2.0, 3.0, 1.0),
    kinematics::Point(2.0, 3.0, 5.0)));

TEST_F(TenJointsChain, ReachAll)
{
  auto point = kinematics::Point(3.0, 5.0, 10.0);
  auto success = chain.goInverse(point);

  std::array<kinematics::Point, 9> expected{
    kinematics::Point(0.50347, -4.9745, 0.0),
    kinematics::Point(0.50347, -4.9745, 0.0),
    kinematics::Point(-0.4429, -5.0152, 4.909),
    kinematics::Point(-0.4429, -5.0152, 4.909),
    kinematics::Point(-0.2058, -0.0217, 4.815),
    kinematics::Point(-0.2058, -0.0217, 4.815),
    kinematics::Point(-1.3628,  2.9578, 8.6602),
    kinematics::Point(-1.3628,  2.9578, 8.6602),
    kinematics::Point(3.0,     5.0,     10.0)
  };

  for (size_t i = 1; i < 10; ++i)
  {
    auto inbetween = chain.goForwardTo(i);

    ASSERT_NEAR(inbetween.x(), expected[i - 1].x(), cmp_point);
    ASSERT_NEAR(inbetween.y(), expected[i - 1].y(), cmp_point);
    ASSERT_NEAR(inbetween.z(), expected[i - 1].z(), cmp_point);
  }
  auto after = chain.goForward();

  EXPECT_TRUE(success);
  ASSERT_NEAR(point.x(), after.x(), cmp_point);
  ASSERT_NEAR(point.y(), after.y(), cmp_point);
  ASSERT_NEAR(point.z(), after.z(), cmp_point);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}