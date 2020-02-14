#include <gtest/gtest.h>
#include <usv_gazebo_plugins/impeller.h>
#include <usv_gazebo_plugins/thrustmapping.h>

using namespace gazebo;

TEST(Impeller, getterAndSetter) {
  Impeller impeller = Impeller(0.0, 0.0);
  ASSERT_FLOAT_EQ(0.0, impeller.getMaxForceFwd());
  ASSERT_FLOAT_EQ(0.0, impeller.getMaxForceRev());
  ASSERT_EQ(ThrustMapping::linear, impeller.getThrustMapping());

  impeller.setMaxForceFwd(1.1);
  ASSERT_FLOAT_EQ(1.1, impeller.getMaxForceFwd());

  impeller.setMaxForceRev(13.0);
  ASSERT_FLOAT_EQ(13.0, impeller.getMaxForceRev());

  impeller.setMaxForceRev(-13.0);
  ASSERT_FLOAT_EQ(13.0, impeller.getMaxForceRev());

  impeller.setThrustMapping(ThrustMapping::GLF);
  ASSERT_EQ(ThrustMapping::GLF, impeller.getThrustMapping());
}

TEST(Impeller, scaleThrustLinear) {
  Impeller impeller = Impeller(1.0, 1.0, ThrustMapping::linear);
  ASSERT_FLOAT_EQ(1.0, impeller.scaleThrust(1.0));
  ASSERT_FLOAT_EQ(-1.0, impeller.scaleThrust(-1.0));
  ASSERT_FLOAT_EQ(0, impeller.scaleThrust(0));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
