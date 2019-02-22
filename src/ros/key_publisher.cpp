#include "include/ur_modern_driver/ros/key_publisher.h"

void KeyPublisher::publishKeyMessage(KeyMessage& data)
{
  emma_commons::URKeyMessage key_msg;
  key_msg.message_title = data.message_title;
  key_msg.text_message = data.text_message;
  key_message_pub_.publish(key_msg);
}

bool KeyPublisher::consume(KeyMessage& data)
{
  LOG_INFO("Got KeyMessage:");
  LOG_INFO("robot message code: %d", km.robot_message_code);
  LOG_INFO("robot message argument: %d", km.robot_message_argument);
  LOG_INFO("message title: %s", km.message_title.c_str());
  LOG_INFO("text message: %s", km.text_message.c_str());

//  publishKeyMessage(data);

  return true;
}
