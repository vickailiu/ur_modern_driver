#include "ur_modern_driver/ros/key_publisher.h"

void KeyPublisher::publishKeyMessage(const KeyMessage& data) const
{
  emma_commons::URKeyMessage key_msg;
  key_msg.message_title = data.message_title;
  key_msg.text_message = data.text_message;
  key_message_pub_.publish(key_msg);
}

bool KeyPublisher::consume(KeyMessage& data)
{
  LOG_INFO("Got KeyMessage:");
  LOG_INFO("robot message code: %d", data.robot_message_code);
  LOG_INFO("robot message argument: %d", data.robot_message_argument);
  LOG_INFO("message title: %s", data.message_title.c_str());
  LOG_INFO("text message: %s", data.text_message.c_str());

//  publishKeyMessage(data);

  return true;
}
