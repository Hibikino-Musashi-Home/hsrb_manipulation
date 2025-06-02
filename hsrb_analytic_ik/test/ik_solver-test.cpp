/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <tmc_manipulation_tests/configs.hpp>

#include "test_common.hpp"

#include <hsrb_analytic_ik/hsrb_ik_solver.hpp>  // NOLINT
#include <hsrb_analytic_ik/hsrc_ik_solver.hpp>  // NOLINT

namespace hsrb_analytic_ik {

class TestDriver {
 public:
  using Ptr = std::shared_ptr<TestDriver>;

  TestDriver(const tmc_robot_kinematics_model::IKSolver::Ptr& analytic_solver,
             const std::string& robot_description,
             tmc_manipulation_types::BaseMovementType base_movement_type)
      : analytic_solver_(analytic_solver) {
    driver_impl_ = std::make_shared<IKTestDriver>(robot_description, base_movement_type);
    switch (base_movement_type) {
      case tmc_manipulation_types::kRotationZ:
        mask_indices_ = {0, 1};
        break;
      default:
        mask_indices_.clear();
        break;
    }
  }

  // Test with random target values, ensuring there is always a solution
  void RunRandomTest() {
    // Restrict to one test in CI due to long test duration
    // this->RunTest("joint_configs/random_config1.dat", "joint_configs/init_config.dat");
    // this->RunTest("joint_configs/random_config2.dat", "joint_configs/init_config.dat");
    // this->RunTest("joint_configs/random_config3.dat", "joint_configs/init_config.dat");
    // this->RunTest("joint_configs/random_config4.dat", "joint_configs/init_config.dat");
    // this->RunTest("joint_configs/random_config5.dat", "joint_configs/init_config.dat");
    // this->RunTest("joint_configs/random_config6.dat", "joint_configs/init_config.dat");
    // this->RunTest("joint_configs/random_config7.dat", "joint_configs/init_config.dat");
    // this->RunTest("joint_configs/random_config8.dat", "joint_configs/init_config.dat");
    // this->RunTest("joint_configs/random_config9.dat", "joint_configs/init_config.dat");
    this->RunTest("joint_configs/random_config10.dat", "joint_configs/init_config.dat");
  }

  // Test with joint angle limit target values, ensuring there is always a solution
  void RunLimitTest(const std::string& config_file) {
    this->RunTest(config_file, "joint_configs/small_init_config.dat");
  }

  // Test with singular target values, ensuring there is always a solution
  void RunSingularTest() {
    this->RunTest("joint_configs/singular_config.dat", "joint_configs/small_init_config.dat");
  }

  // Test with target values that have no solution, which will fail
  void RunUnSolveTest() {
    std::vector<Eigen::VectorXd> init_configs;
    LoadDataFile("joint_configs/small_init_config.dat", init_configs);

    // Loop through initial value patterns
    for (const auto& init_config : init_configs) {
      // z = 2m is unreachable
      auto ref_origin_to_hand = Eigen::Affine3d::Identity();
      ref_origin_to_hand.translation().z() += 2.0;

      Eigen::VectorXd analytic_solution(8);
      EXPECT_FALSE(driver_impl_->SolveIK(analytic_solver_, ref_origin_to_hand, init_config, analytic_solution));
    }
  }

 private:
  void RunTest(const std::string& ref_config_file, const std::string& init_config_file) {
    std::vector<Eigen::VectorXd> ref_configs;
    LoadDataFile(ref_config_file, ref_configs);

    std::vector<Eigen::VectorXd> init_configs;
    LoadDataFile(init_config_file, init_configs);

    hsrb_analytic_ik::RunTest(ref_configs, init_configs, driver_impl_, analytic_solver_, mask_indices_);
  }

  tmc_robot_kinematics_model::IKSolver::Ptr analytic_solver_;
  IKTestDriver::Ptr driver_impl_;
  std::vector<uint32_t> mask_indices_;
};


class HsrbIKSolverTest : public ::testing::Test {
 protected:
  HsrbIKSolverTest() {
    tmc_robot_kinematics_model::IKSolver::Ptr solver(new hsrb_analytic_ik::HsrbIKSolver());
    const auto robot_description = tmc_manipulation_tests::hsrb::GetUrdf();
    test_driver_ = std::make_shared<TestDriver>(solver, robot_description, tmc_manipulation_types::kPlanar);
  }

  TestDriver::Ptr test_driver_;
};

TEST_F(HsrbIKSolverTest, RandomTest) { test_driver_->RunRandomTest(); }
TEST_F(HsrbIKSolverTest, LimitTest) { test_driver_->RunLimitTest("joint_configs/limit_config.dat"); }
TEST_F(HsrbIKSolverTest, SingularTest) { test_driver_->RunSingularTest(); }
TEST_F(HsrbIKSolverTest, UnSolveTest) { test_driver_->RunUnSolveTest(); }

class HsrcIKSolverTest : public ::testing::Test {
 protected:
  HsrcIKSolverTest() {
    tmc_robot_kinematics_model::IKSolver::Ptr solver(new hsrb_analytic_ik::HsrcIKSolver());
    const auto robot_description = tmc_manipulation_tests::hsrc::GetUrdf();
    test_driver_ = std::make_shared<TestDriver>(solver, robot_description, tmc_manipulation_types::kPlanar);
  }

  TestDriver::Ptr test_driver_;
};

TEST_F(HsrcIKSolverTest, RandomTest) { test_driver_->RunRandomTest(); }
TEST_F(HsrcIKSolverTest, LimitTest) { test_driver_->RunLimitTest("joint_configs/limit_config_501.dat"); }
TEST_F(HsrcIKSolverTest, SingularTest) { test_driver_->RunSingularTest(); }
TEST_F(HsrcIKSolverTest, UnSolveTest) { test_driver_->RunUnSolveTest(); }

class HsrbBaseYawIKSolverTest : public ::testing::Test {
 protected:
  HsrbBaseYawIKSolverTest() {
    tmc_robot_kinematics_model::IKSolver::Ptr solver(new hsrb_analytic_ik::HsrbBaseYawIKSolver());
    const auto robot_description = tmc_manipulation_tests::hsrb::GetUrdf();
    test_driver_ = std::make_shared<TestDriver>(solver, robot_description, tmc_manipulation_types::kRotationZ);
  }

  TestDriver::Ptr test_driver_;
};

TEST_F(HsrbBaseYawIKSolverTest, RandomTest) { test_driver_->RunRandomTest(); }
TEST_F(HsrbBaseYawIKSolverTest, LimitTest) { test_driver_->RunLimitTest("joint_configs/limit_config.dat"); }
TEST_F(HsrbBaseYawIKSolverTest, SingularTest) { test_driver_->RunSingularTest(); }
TEST_F(HsrbBaseYawIKSolverTest, UnSolveTest) { test_driver_->RunUnSolveTest(); }

class HsrcBaseYawIKSolverTest : public ::testing::Test {
 protected:
  HsrcBaseYawIKSolverTest() {
    tmc_robot_kinematics_model::IKSolver::Ptr solver(new hsrb_analytic_ik::HsrcBaseYawIKSolver());
    const auto robot_description = tmc_manipulation_tests::hsrc::GetUrdf();
    test_driver_ = std::make_shared<TestDriver>(solver, robot_description, tmc_manipulation_types::kRotationZ);
  }

  TestDriver::Ptr test_driver_;
};

TEST_F(HsrcBaseYawIKSolverTest, RandomTest) { test_driver_->RunRandomTest(); }
TEST_F(HsrcBaseYawIKSolverTest, LimitTest) { test_driver_->RunLimitTest("joint_configs/limit_config_501.dat"); }
TEST_F(HsrcBaseYawIKSolverTest, SingularTest) { test_driver_->RunSingularTest(); }
TEST_F(HsrcBaseYawIKSolverTest, UnSolveTest) { test_driver_->RunUnSolveTest(); }

// Test for obtaining multiple solutions, focusing on HSR-B as a representative example
TEST(HsrbBaseYawIKSolverMultiSolutionTest, MultiSolution) {
  const auto robot_description = tmc_manipulation_tests::hsrb::GetUrdf();
  auto driver = std::make_shared<IKTestDriver>(robot_description, tmc_manipulation_types::kRotationZ);

  auto solver = std::make_shared<hsrb_analytic_ik::HsrbBaseYawIKSolver>();

  // Although there can be up to 4 solutions, relevant end-effector positions and orientations are included
  MultiSolutionTest(Eigen::Translation3d(0.6, 0.07, 0.6) * Eigen::Quaterniond(0.0, 0.707, 0.0, 0.707),
                    4, driver, solver);
}

// Calculate the range of mobile base positions, HSR-B
TEST(BasePositionRangeTest, GetHsrbBasePositionRange) {
  // Test data is created by viewing the robot model in rviz, allowing for centimeter-level deviations
  constexpr double kEpsilon = 1.0e-2;
  constexpr double kCenterX = -0.14;
  constexpr double kCenterY = 0.0;
  // End-effector in a horizontal posture
  const auto rot = Eigen::Quaterniond(0.0, 0.707, 0.0, 0.707);
  {
    // Too high to have a solution; a negative radius value indicates no solution
    const auto range = GetHsrbBasePositionRange(Eigen::Translation3d(0.0, 0.0, 1.4) * rot);
    EXPECT_LT(range.radius_min, 0.0);
  }
  {
    // Too low to have a solution; a negative radius value indicates no solution
    const auto range = GetHsrbBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.0) * rot);
    EXPECT_LT(range.radius_min, 0.0);
  }
  {
    // When aiming to place the mobile base as far as possible, it's too low to stretch out the arm straight
    const auto range = GetHsrbBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.1) * rot);
    // Check only twice since the posture is the same but height varies, results should be consistent
    EXPECT_NEAR(range.center[0], kCenterX, kEpsilon);
    EXPECT_NEAR(range.center[1], kCenterY, kEpsilon);
    EXPECT_NEAR(range.radius_max, 0.40, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // Height at which the arm can be stretched out horizontally when aiming to place the mobile base as far as possible
    const auto range = GetHsrbBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.35) * rot);
    EXPECT_NEAR(range.center[0], kCenterX, kEpsilon);
    EXPECT_NEAR(range.center[1], kCenterY, kEpsilon);
    EXPECT_NEAR(range.radius_max, 0.49, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // Height at which the arm can be stretched out horizontally when aiming to place the mobile base as far as possible, version 2
    const auto range = GetHsrbBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.95) * rot);
    EXPECT_NEAR(range.radius_max, 0.49, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // When aiming to place the mobile base as far as possible, it's too high to stretch out the arm straight
    const auto range = GetHsrbBasePositionRange(Eigen::Translation3d(0.0, 0.0, 1.2) * rot);
    EXPECT_NEAR(range.radius_max, 0.44, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // When aiming to place the mobile base as far as possible, it's better to lower the arm to its limit
    const auto range = GetHsrbBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.2) * rot);
    EXPECT_NEAR(range.radius_min, 0.32, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // When aiming to place the mobile base as far as possible, lowering the lift axis and extending the arm forward is preferable
    const auto range = GetHsrbBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.55) * rot);
    EXPECT_NEAR(range.radius_min, 0.41, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // Height at which the arm can be folded when aiming to place the mobile base as far as possible
    const auto range = GetHsrbBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.7) * rot);
    EXPECT_NEAR(range.radius_min, 0.16, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
}

// Calculate the range of mobile base positions, HSR-C
TEST(BasePositionRangeTest, GetHsrcBasePositionRange) {
  // Test data is created by viewing the robot model in rviz, allowing for centimeter-level deviations
  constexpr double kEpsilon = 1.0e-2;
  constexpr double kCenterX = -0.15;
  constexpr double kCenterY = 0.0;
  // End-effector in a horizontal posture
  const auto rot = Eigen::Quaterniond(0.0, 0.707, 0.0, 0.707);
  {
    // Too high to have a solution; a negative radius value indicates no solution
    const auto range = GetHsrcBasePositionRange(Eigen::Translation3d(0.0, 0.0, 1.4) * rot);
    EXPECT_LT(range.radius_min, 0.0);
  }
  {
    // Too low to have a solution; a negative radius value indicates no solution
    const auto range = GetHsrcBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.0) * rot);
    EXPECT_LT(range.radius_min, 0.0);
  }
  {
    // When aiming to place the mobile base as far as possible, it's too low to stretch out the arm straight
    const auto range = GetHsrcBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.1) * rot);
    // Check only twice since the posture is the same but height varies, results should be consistent
    EXPECT_NEAR(range.center[0], kCenterX, kEpsilon);
    EXPECT_NEAR(range.center[1], kCenterY, kEpsilon);
    EXPECT_NEAR(range.radius_max, 0.38, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // Height at which the arm can be stretched out horizontally when aiming to place the mobile base as far as possible
    const auto range = GetHsrcBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.35) * rot);
    EXPECT_NEAR(range.center[0], kCenterX, kEpsilon);
    EXPECT_NEAR(range.center[1], kCenterY, kEpsilon);
    EXPECT_NEAR(range.radius_max, 0.49, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // Height at which the arm can be stretched out horizontally when aiming to place the mobile base as far as possible, version 2
    const auto range = GetHsrcBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.95) * rot);
    EXPECT_NEAR(range.radius_max, 0.49, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // When aiming to place the mobile base as far as possible, it's too high to stretch out the arm straight
    const auto range = GetHsrcBasePositionRange(Eigen::Translation3d(0.0, 0.0, 1.2) * rot);
    EXPECT_NEAR(range.radius_max, 0.45, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // When aiming to place the mobile base as far as possible, it's better to lower the arm to its limit
    const auto range = GetHsrcBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.2) * rot);
    EXPECT_NEAR(range.radius_min, 0.32, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // When aiming to place the mobile base as far as possible, lowering the lift axis and extending the arm forward is preferable
    const auto range = GetHsrcBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.55) * rot);
    EXPECT_NEAR(range.radius_min, 0.43, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
  {
    // Height at which the arm can be folded when aiming to place the mobile base as far as possible
    const auto range = GetHsrcBasePositionRange(Eigen::Translation3d(0.0, 0.0, 0.7) * rot);
    EXPECT_NEAR(range.radius_min, 0.16, kEpsilon);
    EXPECT_LT(range.radius_min, range.radius_max);
  }
}

}  // namespace hsrb_analytic_ik

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
