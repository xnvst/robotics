/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * Copyright 2015 Avidbots Corp.
 * @name    SafetyZoneTest
 * @brief   Tests for the safety monitor safety zone
 * @author  Tony Lu
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>

#include "avidbots_safety_monitor/safety_monitor_safetyzone.h"
#include "avidbots_safety_monitor/safety_monitor_helper.h"
#include <avidbots_library/get_param/get_param_util.h>
#include <avidbots_library/safety_zone/properties.h>
#include <avidbots_library/geometry/geometry_msgs/polygon.h>
#include <avidbots_library/safety_zone/polygon.h>

class SafetyMonitorSafetyZone;

class SafetyZoneTest : public ::testing::Test
{
protected:
  static Footprint footprint_;
  static double ang_vel_const_;
  static double safety_dist_mult_;
  static double gear_ratio_;
  static double wheel_radius_;
  static double max_rpm_speed_;
  static double corner_delta_x_;
  static double corner_delta_y_;
  static std::string laser_name_;
  static LaserInfo laser_;
  static geometry_msgs::PolygonStamped footprint_vertices_;

  ros::NodeHandle private_node_handle;
  SafetyMonitorSafetyZone safety_zone_;
  std::vector<Point> polygon_points_;
  double laser_delta_forward_ = 0.661600;

  double kRPMPerSecToRadPerSecSquared = 2 * M_PI / 60;
  double motor_accel = max_rpm_speed_ * kRPMPerSecToRadPerSecSquared / gear_ratio_;
  double max_accel_ = motor_accel * wheel_radius_;

  /* Test related */
  void SetUp();
  static void SetUpTestCase()
  {
    ros::Time::init();
    GetParamUtil::GetFootprintParam("/footprint", footprint_vertices_,
                                  GetParamUtil::GetDefaultFootprintPolygon());
    geometry_msgs::util::FootprintProperties footprint_properties(footprint_vertices_);
    footprint_.positive_max_x_position = footprint_properties.max_x;
    GetParamUtil::GetParam("/robot_properties/footprint_length", footprint_.length, .914);
    GetParamUtil::GetParam("/robot_properties/footprint_width", footprint_.width, .68);
    GetParamUtil::GetParam("/sm_properties/angular_velocity_constant", ang_vel_const_, 0.5);
    GetParamUtil::GetParam("/sm_properties/safety_distance_multiplier", safety_dist_mult_, 1);
    GetParamUtil::GetParam("/robot_properties/wheel_radius", wheel_radius_, 0.1016);
    GetParamUtil::GetParam("/robot_properties/motor_controller_max_accel", max_rpm_speed_, 3000);
    GetParamUtil::GetParam("/robot_properties/gear_ratio", gear_ratio_, 44.4);
    GetParamUtil::GetParam("/robot_properties/laser_link", laser_name_, "/hokuyo_ust_20lx_f_laser_link");
    laser_.delta_forward = 0.661600;
  }

  /* Helper Functions */
  void CreateInitialZone();
  void InitLaserTransform();
  void AddExpZonePadding();
  void CheckSolution(std::vector<Point>& a, std::vector<Point>& b);
};

/* Static members */
Footprint SafetyZoneTest::footprint_;
double SafetyZoneTest::ang_vel_const_;
double SafetyZoneTest::safety_dist_mult_;
double SafetyZoneTest::gear_ratio_;
double SafetyZoneTest::wheel_radius_;
double SafetyZoneTest::max_rpm_speed_;
double SafetyZoneTest::corner_delta_x_;
double SafetyZoneTest::corner_delta_y_;
std::string SafetyZoneTest::laser_name_;
LaserInfo SafetyZoneTest::laser_;
geometry_msgs::PolygonStamped SafetyZoneTest::footprint_vertices_;

//////////////////////////////////////////////
/// Test Setup
//////////////////////////////////////////////

/**
 * @name    SetUp()
 * @brief   Prepares member fields for each test.
 */
void SafetyZoneTest::SetUp()
{
  CreateInitialZone();
  safety_zone_.Init(private_node_handle, false);
  safety_zone_.SetLaserInfo(laser_);
}

//////////////////////////////////////////////
/// Helpers
//////////////////////////////////////////////

/**
 * @name    CreateInitialZone()
 * @brief   Creates the initial safety zone without padding
 */
void SafetyZoneTest::CreateInitialZone()
{
  polygon_points_ = safety_monitor_helper::CreatePolygon(footprint_vertices_, 0.0);
}

/**
 * @name    AddExpZonePadding()
 * @brief   Adds a padding to the expanding safety zone
 */
void SafetyZoneTest::AddExpZonePadding()
{
  double padding = safety_zone_.kExpZoneOffset;
  polygon_points_ = Polygon::AddPolygonOffset(polygon_points_, padding, 0.0);
}

/**
 * @name    CheckSolution()
 * @brief   Checks if the scalar values of points in each vector are equal
 * param[in] a: Expected vector of points
 * param[in] b: Actual vector of points
 */
void SafetyZoneTest::CheckSolution(std::vector<Point>& a, std::vector<Point>& b)
{
  for (int i = 0; i< polygon_points_.size(); i++)
  {
    EXPECT_NEAR(a.at(i).x, b.at(i).x, 0.0001);
    EXPECT_NEAR(a.at(i).y, b.at(i).y, 0.0001);
  }
}

//////////////////////////////////////////////
/// Testing Fixtures
//////////////////////////////////////////////

/*
 * @brief  test for the safety danger zone
 * Expected: Zone has only danger padding
 */
TEST_F(SafetyZoneTest, DangerZone)
{
  safety_zone_.CreateDangerZone();
  std::vector<Point> danger_polygon_points = safety_zone_.DangerPolygonPoints();

  CheckSolution(polygon_points_, danger_polygon_points);
}

/*
 * @brief  test for the expanding safety zone with zero velocity
 * Expected: Zone does not expand
 */
TEST_F(SafetyZoneTest, ExpZoneZeroVelocity)
{
  Point vector;
  vector.x = 0;
  vector.y = 0;
  double ang_vel = 0;

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  test for the expanding safety zone when
 *         velocity.x = 1.0 m/s
 *         velocity.y = 1.0 m/s
 *         angular_velocity = 0.0 m/s
 * Expected: Zone expands horizontally and vertically
             in the positive x and y-axis
 */
TEST_F(SafetyZoneTest, ExpZoneNonZeroVelocity)
{
  Point vector;
  vector.x = 1.0;
  vector.y = 1.0;
  double ang_vel = 0;
  double stop_dist = safety_monitor_helper::StoppingDistance(vector, max_accel_, safety_dist_mult_);

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();
  polygon_points_.at(1).y += stop_dist*sin(M_PI/4);
  for (int i=2; i < polygon_points_.size(); i++)
  {
    polygon_points_.at(i).x += stop_dist*cos(M_PI/4);
    polygon_points_.at(i).y += stop_dist*sin(M_PI/4);
  }

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  test for the expanding safety zone when
 *         velocity.x = 2.0 m/s
 *         velocity.y = 0.0 m/s
 *         angular_velocity = 0.0 m/s
 * Expected: Zone expands horizontally in the positive x-axis
 */
TEST_F(SafetyZoneTest, ExpZoneNonZeroVelocity2)
{
  Point vector;
  vector.x = 2.0;
  vector.y = 0.0;
  double ang_vel = 0;
  double stop_dist = safety_monitor_helper::StoppingDistance(vector, max_accel_, safety_dist_mult_);

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();
  for (int i=0; i < polygon_points_.size(); i++)
  {
    if (i != 0 && i != 1)
    {
      polygon_points_.at(i).x += stop_dist*cos(0);
    }
  }

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  test for the expanding safety zone when
 *         velocity.x = 0.0 m/s
 *         velocity.y = 1.8 m/s
 *         angular_velocity = 0.0 m/s
 * Expected: Zone expands vertically in the positive y-axis
 */
TEST_F(SafetyZoneTest, ExpZoneNonZeroVelocity3)
{
  Point vector;
  vector.x = 0.0;
  vector.y = 1.8;
  double ang_vel = 0;
  double stop_dist = safety_monitor_helper::StoppingDistance(vector, max_accel_, safety_dist_mult_);

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();
  int end = polygon_points_.size()/2 + 1;
  for (int i=1; i < end; i++)
  {
    polygon_points_.at(i).y += stop_dist*sin(M_PI/2);
  }

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  test for the expanding safety zone when
 *         velocity.x = 0.0 m/s
 *         velocity.y = -1.8 m/s
 *         angular_velocity = 0.0 m/s
 * Expected: Zone expands vertically in the negative y-axis
 */
TEST_F(SafetyZoneTest, ExpZoneNonZeroVelocity4)
{
  Point vector;
  vector.x = 0.0;
  vector.y = -1.8;
  double ang_vel = 0;
  double stop_dist = safety_monitor_helper::StoppingDistance(vector, max_accel_, safety_dist_mult_);

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();
  polygon_points_.at(0).y -= stop_dist*sin(M_PI/2);
  int start = polygon_points_.size()/2 + 1;
  for (int i=start; i < polygon_points_.size(); i++)
  {
    polygon_points_.at(i).y -= stop_dist*sin(M_PI/2);
  }

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  test for the expanding safety zone when
 *         velocity.x = 1.5 m/s
 *         velocity.y = 0.0 m/s
 *         angular_velocity = 0.09 m/s
 * Expected: Zone expands horizontally in the positive x-axis and
 *           does not rotate because rotation speed is too small
 */
TEST_F(SafetyZoneTest, ExpZoneNonZeroVelocityNonZeroRotation)
{
  Point vector;
  vector.x = 1.5;
  vector.y = 0.0;
  double ang_vel = 0.09;
  double stop_dist = safety_monitor_helper::StoppingDistance(vector, max_accel_, safety_dist_mult_);

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();
  for (int i=2; i < polygon_points_.size(); i++)
  {
    polygon_points_.at(i).x += stop_dist*cos(0);
  }

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  test for the expanding safety zone when
 *         velocity.x = 2.0 m/s
 *         velocity.y = 0.0 m/s
 *         angular_velocity = 0.3 m/s
 * Expected: Zone expands horizontally in the positive x-axis and rotates CCW
 */
TEST_F(SafetyZoneTest, ExpZoneNonZeroVelocityNonZeroRotation2)
{
  Point vector;
  vector.x = 2.0;
  vector.y = 0.0;
  double ang_vel = 0.3;
  double stop_dist = safety_monitor_helper::StoppingDistance(vector, max_accel_, safety_dist_mult_);

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();
  for (int i=2; i < polygon_points_.size(); i++)
  {
    polygon_points_.at(i).x += stop_dist*cos(0);
  }
  polygon_points_ = Polygon::RotatePolygon(ang_vel * ang_vel_const_, polygon_points_, 0.0);

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  test for the expanding safety zone when
 *         velocity.x = 1.2 m/s
 *         velocity.y = 0.4 m/s
 *         angular_velocity = 0.3 m/s
 * Expected: Zone expands horizontally in the positive x-axis and
 *           vertically in the positive y-axis and rotates CCW
 */
TEST_F(SafetyZoneTest, ExpZoneNonZeroVelocityNonZeroRotation3)
{
  Point vector;
  vector.x = 1.2;
  vector.y = 0.4;
  double ang_vel = 0.3;
  double ang = atan2(vector.y, vector.x);
  double stop_dist = safety_monitor_helper::StoppingDistance(vector, max_accel_, safety_dist_mult_);

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();
  polygon_points_.at(1).y += stop_dist*sin(ang);
  for (int i=2; i < polygon_points_.size(); i++)
  {
    polygon_points_.at(i).x += stop_dist*cos(ang);
    polygon_points_.at(i).y += stop_dist*sin(ang);
  }
  polygon_points_ = Polygon::RotatePolygon(ang_vel * ang_vel_const_, polygon_points_, 0.0);

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  test for the expanding safety zone when
 *         velocity.x = 1.2 m/s
 *         velocity.y = -0.4 m/s
 *         angular_velocity = -0.3 m/s
 * Expected: Zone expands horizontally in the positive x-axis and
 *           vertically in the negative y-axis and rotates CW
 */
TEST_F(SafetyZoneTest, ExpZoneNonZeroVelocityNonZeroRotation4)
{
  Point vector;
  vector.x = 1.2;
  vector.y = -0.4;
  double ang_vel = -0.3;
  double ang = atan2(vector.y, vector.x);
  double stop_dist = safety_monitor_helper::StoppingDistance(vector, max_accel_, safety_dist_mult_);

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();
  polygon_points_.at(0).y -= fabs(stop_dist*sin(ang));
  for (int i=2; i < polygon_points_.size(); i++)
  {
    polygon_points_.at(i).x += stop_dist*cos(ang);
    polygon_points_.at(i).y -= fabs(stop_dist*sin(ang));
  }
  polygon_points_ = Polygon::RotatePolygon(ang_vel * ang_vel_const_, polygon_points_, 0.0);

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  test for the expanding safety zone when
 *         velocity.x = -1.2 m/s
 *         velocity.y = 0.0 m/s
 *         angular_velocity = 0.0 m/s
 * Expected: Zone does not change from inital zone because
 *           the forward vertices cannot go behind the base vertices
 */
TEST_F(SafetyZoneTest, ExpZoneBehindBase)
{
  Point vector;
  vector.x = -1.2;
  vector.y = 0.0;
  double ang_vel = 0.0;
  double stop_dist = safety_monitor_helper::StoppingDistance(vector, max_accel_, safety_dist_mult_);

  /* Actual Calculations */
  safety_zone_.CreateExpSafetyZone(vector, ang_vel);
  std::vector<Point> exp_polygon_points = safety_zone_.ExpPolygonPoints();

  /* Expected Calculations */
  AddExpZonePadding();

  CheckSolution(polygon_points_, exp_polygon_points);
}

/*
 * @brief  Main function that runs all tests
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "SafetyZoneTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
