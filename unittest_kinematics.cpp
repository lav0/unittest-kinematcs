#include "gtest\gtest.h"
#include "../NelderMeadSolver/SimplexFactory.h"
#include "../NelderMeadSolver/Solver.h"
#include "../KinematicBase/ValueWrapperJoint.h"
#include "../KinematicBase/RotaryJoint.h"
#include "../KinematicBase/LinearJoint.h"
#include "../KinematicBase/Chain.h"


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
  ASSERT_NEAR(point.x(), after.x(), 0.1);
  ASSERT_NEAR(point.y(), after.y(), 0.1);
  ASSERT_NEAR(point.z(), after.z(), 0.1);
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
  ASSERT_NEAR(point.x(), after.x(), 0.1);
  ASSERT_NEAR(point.y(), after.y(), 0.1);
  ASSERT_NEAR(point.z(), after.z(), 0.1);
  ASSERT_NEAR(expec1, j1->getJointValue(), 0.1);
  ASSERT_NEAR(expec2, j2->getJointValue(), 0.1);
}

INSTANTIATE_TEST_CASE_P(Inst, TwoJointsChain,
  ::testing::Values(
    PointAndTwoDoubles{ kinematics::Point(0.0, 5.0, 0.0), 0.523602, 2.09439 },
    PointAndTwoDoubles{ kinematics::Point(0.0, 3.0, 0.0), 2.83689999957439, -2.53221 }
));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}