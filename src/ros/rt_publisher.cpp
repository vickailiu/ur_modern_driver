#include "ur_modern_driver/ros/rt_publisher.h"

bool RTPublisher::publishJoints(RTShared& packet, Time& t)
{
  sensor_msgs::JointState joint_msg;
  joint_msg.header.stamp = t;

  joint_msg.name.assign(joint_names_.begin(), joint_names_.end());
  joint_msg.position.assign(packet.q_actual.begin(), packet.q_actual.end());
  joint_msg.velocity.assign(packet.qd_actual.begin(), packet.qd_actual.end());
  joint_msg.effort.assign(packet.i_actual.begin(), packet.i_actual.end());

  joint_pub_.publish(joint_msg);

  return true;
}

bool RTPublisher::publishWrench(RTShared& packet, Time& t)
{
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = t;
  wrench_msg.wrench.force.x = packet.tcp_force[0];
  wrench_msg.wrench.force.y = packet.tcp_force[1];
  wrench_msg.wrench.force.z = packet.tcp_force[2];
  wrench_msg.wrench.torque.x = packet.tcp_force[3];
  wrench_msg.wrench.torque.y = packet.tcp_force[4];
  wrench_msg.wrench.torque.z = packet.tcp_force[5];

  wrench_pub_.publish(wrench_msg);
  return true;
}

bool RTPublisher::publishTool(RTShared& packet, Time& t)
{
  geometry_msgs::TwistStamped tool_twist;
  tool_twist.header.stamp = t;
  tool_twist.header.frame_id = base_frame_;
  tool_twist.twist.linear.x = packet.tcp_speed_actual.position.x;
  tool_twist.twist.linear.y = packet.tcp_speed_actual.position.y;
  tool_twist.twist.linear.z = packet.tcp_speed_actual.position.z;
  tool_twist.twist.angular.x = packet.tcp_speed_actual.rotation.x;
  tool_twist.twist.angular.y = packet.tcp_speed_actual.rotation.y;
  tool_twist.twist.angular.z = packet.tcp_speed_actual.rotation.z;

  tool_vel_pub_.publish(tool_twist);
  return true;
}

bool RTPublisher::publishToolVector(RTShared& packet, Time& t)
{
  auto tv_actual = packet.tool_vector_actual;
  auto tv_target = packet.tool_vector_target;
  emma_commons::URToolMessage tool_vector;

  double angle_actual = std::sqrt(std::pow(tv_actual.rotation.x, 2) + std::pow(tv_actual.rotation.y, 2) + std::pow(tv_actual.rotation.z, 2));
  double angle_target = std::sqrt(std::pow(tv_target.rotation.x, 2) + std::pow(tv_target.rotation.y, 2) + std::pow(tv_target.rotation.z, 2));
  
  tool_vector.header.stamp = t;
  tool_vector.target_position.x = packet.tool_vector_target.position.x;
  tool_vector.target_position.y = packet.tool_vector_target.position.y;
  tool_vector.target_position.z = packet.tool_vector_target.position.z;
  if (angle_target < 1e-16)
  {
    tool_vector.target_rotation.x = 0;
    tool_vector.target_rotation.y = 0;
    tool_vector.target_rotation.z = 0;
  }
  else
  {
    tool_vector.target_rotation.x = packet.tool_vector_target.rotation.x / angle_target;
    tool_vector.target_rotation.y = packet.tool_vector_target.rotation.y / angle_target;
    tool_vector.target_rotation.z = packet.tool_vector_target.rotation.z / angle_target;
  }

  tool_vector.actual_position.x = packet.tool_vector_actual.position.x;
  tool_vector.actual_position.y = packet.tool_vector_actual.position.y;
  tool_vector.actual_position.z = packet.tool_vector_actual.position.z;
  if (angle_target < 1e-16)
  {
    tool_vector.actual_rotation.x = 0;
    tool_vector.actual_rotation.y = 0;
    tool_vector.actual_rotation.z = 0;
  }
  else
  {
    tool_vector.actual_rotation.x = packet.tool_vector_actual.rotation.x / angle_actual;
    tool_vector.actual_rotation.y = packet.tool_vector_actual.rotation.y / angle_actual;
    tool_vector.actual_rotation.z = packet.tool_vector_actual.rotation.z / angle_actual;
  }

  tool_vector_pub_.publish(tool_vector);
  return true;
}

bool RTPublisher::publishPosition(RTShared& packet, Time& t)
{
  emma_commons::URPositionMessage position_msg;
  position_msg.header.stamp = t;
  position_msg.target_position.assign(packet.q_target.begin(), packet.q_target.end());
  position_msg.actual_position.assign(packet.q_actual.begin(), packet.q_actual.end());

  position_pub_.publish(position_msg);
  return true;
}

bool RTPublisher::publishCurrent(RTShared& packet, Time& t)
{
  emma_commons::URCurrentMessage current_msg;
  current_msg.header.stamp = t;
  current_msg.target_current.assign(packet.i_target.begin(), packet.i_target.end());
  current_msg.actual_current.assign(packet.i_actual.begin(), packet.i_actual.end());

  current_pub_.publish(current_msg);
  return true;
}

bool RTPublisher::publishTransform(RTShared& packet, Time& t)
{
  auto tv = packet.tool_vector_actual;

  Transform transform;
  transform.setOrigin(Vector3(tv.position.x, tv.position.y, tv.position.z));

  // Create quaternion
  Quaternion quat;

  double angle = std::sqrt(std::pow(tv.rotation.x, 2) + std::pow(tv.rotation.y, 2) + std::pow(tv.rotation.z, 2));
  if (angle < 1e-16)
  {
    quat.setValue(0, 0, 0, 1);
  }
  else
  {
    quat.setRotation(Vector3(tv.rotation.x / angle, tv.rotation.y / angle, tv.rotation.z / angle), angle);
  }

  transform.setRotation(quat);

  transform_broadcaster_.sendTransform(StampedTransform(transform, t, base_frame_, tool_frame_));

  return true;
}

bool RTPublisher::publishTemperature(RTShared& packet, Time& t)
{
  size_t len = joint_names_.size();
  for (size_t i = 0; i < len; i++)
  {
    sensor_msgs::Temperature msg;
    msg.header.stamp = t;
    msg.header.frame_id = joint_names_[i];
    msg.temperature = packet.motor_temperatures[i];

    joint_temperature_pub_.publish(msg);
  }
  return true;
}

bool RTPublisher::publish(RTShared& packet)
{
  Time time = Time::now();
  bool res = true;
  if (!temp_only_)
  {
    res = publishJoints(packet, time) && publishWrench(packet, time) && publishTool(packet, time) &&
          publishTransform(packet, time) && publishToolVector(packet, time) && publishPosition(packet, time) && publishCurrent(packet, time);
  }

  return res && publishTemperature(packet, time);
}

bool RTPublisher::consume(RTState_V1_6__7& state)
{
  return publish(state);
}
bool RTPublisher::consume(RTState_V1_8& state)
{
  return publish(state);
}
bool RTPublisher::consume(RTState_V3_0__1& state)
{
  return publish(state);
}
bool RTPublisher::consume(RTState_V3_2__3& state)
{
  return publish(state);
}
