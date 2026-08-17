#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "wiln/PathDeformer.hpp"
#include "wiln/RobotModel.hpp"

namespace
{

nav_msgs::msg::Path straightPath(double length, double spacing)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x = 0.0; x <= length + 1e-9; x += spacing) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(PathDeformer, CentralObstacleProducesOneSidedSmoothFeasibleBypass)
{
  wiln::PathDeformer::Params params;
  params.attraction_gain = 0.8;
  params.repulsion_gain = 1.4;
  params.repulsion_dist = 2.0;
  params.internal_force = 4.0;
  params.influence_longitudinal = 3.0;
  params.obstacle_margin = 0.0;
  params.max_iterations = 50;
  params.step_size = 1.0;
  params.max_deformation_step = 0.12;
  params.max_total_deformation = 1.5;
  params.time_budget_ms = 1000.0;

  wiln::PathDeformer deformer(params);
  wiln::GenericModel::Params model_params;
  model_params.width = 1.0;
  model_params.kappa_max = 0.7;
  wiln::GenericModel model(model_params);
  deformer.setRobotModel(&model);

  std::vector<Eigen::Vector3d> obstacles;
  for (double y = -0.1; y <= 0.1; y += 0.05) {
    obstacles.emplace_back(5.0, y, 0.0);
  }

  wiln::PathDeformer::Diag diag;
  const auto deformed = deformer.deform(straightPath(10.0, 0.2), obstacles, &diag);

  ASSERT_FALSE(diag.used_fallback)
    << "clearance=" << diag.min_clearance_m
    << " curvature=" << diag.max_curvature_m_inv
    << " displacement=" << diag.max_displacement_m;
  ASSERT_TRUE(diag.path_is_clear);
  EXPECT_EQ(diag.avoidance_side, -1);
  EXPECT_LE(diag.max_curvature_m_inv, model.kappaMax() * 1.05);

  int sign_changes = 0;
  int previous_sign = 0;
  for (const auto & pose : deformed.poses) {
    const double y = pose.pose.position.y;
    if (std::abs(y) < 1e-3) {
      continue;
    }
    const int sign = y > 0.0 ? 1 : -1;
    if (previous_sign != 0 && sign != previous_sign) {
      ++sign_changes;
    }
    previous_sign = sign;
  }
  EXPECT_EQ(sign_changes, 0);
}

TEST(PathDeformer, ImpossibleCorridorFallsBackInsteadOfClippingPoints)
{
  wiln::PathDeformer::Params params;
  params.repulsion_gain = 1.2;
  params.repulsion_dist = 2.0;
  params.influence_longitudinal = 3.0;
  params.obstacle_margin = 0.25;
  params.max_total_deformation = 0.4;
  params.time_budget_ms = 1000.0;

  wiln::PathDeformer deformer(params);
  wiln::GenericModel model;
  deformer.setRobotModel(&model);
  const std::vector<Eigen::Vector3d> obstacles{{5.0, 0.0, 0.0}};

  wiln::PathDeformer::Diag diag;
  const auto original = straightPath(10.0, 0.2);
  const auto result = deformer.deform(original, obstacles, &diag);

  ASSERT_TRUE(diag.used_fallback);
  ASSERT_FALSE(diag.path_is_clear);
  ASSERT_EQ(result.poses.size(), original.poses.size());
  for (size_t i = 0; i < result.poses.size(); ++i) {
    EXPECT_DOUBLE_EQ(result.poses[i].pose.position.x, original.poses[i].pose.position.x);
    EXPECT_DOUBLE_EQ(result.poses[i].pose.position.y, original.poses[i].pose.position.y);
  }
}

TEST(PathDeformer, ProductionMttParametersYieldFeasibleEarlyBypass)
{
  wiln::PathDeformer::Params params;
  params.attraction_gain = 0.8;
  params.repulsion_gain = 1.2;
  params.repulsion_dist = 3.5;
  params.internal_force = 4.0;
  params.influence_longitudinal = 6.0;
  params.obstacle_margin = 0.25;
  params.max_iterations = 40;
  params.step_size = 1.0;
  params.max_deformation_step = 0.12;
  params.max_total_deformation = 2.25;
  params.time_budget_ms = 1000.0;

  wiln::PathDeformer deformer(params);
  wiln::MttModel::Params model_params;
  model_params.kappa_max_nominal = 0.35;
  model_params.tractor_width = 2.5;
  model_params.trailer_width = 2.5;
  wiln::MttModel model(model_params);
  deformer.setRobotModel(&model);

  std::vector<Eigen::Vector3d> obstacles;
  for (double y = -0.1; y <= 0.1; y += 0.05) {
    obstacles.emplace_back(7.0, y, 0.0);
  }
  wiln::PathDeformer::Diag diag;
  (void)deformer.deform(straightPath(15.0, 0.05), obstacles, &diag);

  ASSERT_FALSE(diag.used_fallback)
    << "clearance=" << diag.min_clearance_m
    << " curvature=" << diag.max_curvature_m_inv
    << " displacement=" << diag.max_displacement_m;
  EXPECT_TRUE(diag.path_is_clear);
  EXPECT_GE(diag.min_clearance_m, 1.5);
  EXPECT_LE(diag.max_curvature_m_inv, 0.35 * 1.05);
}
