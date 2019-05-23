#pragma once

#include "emma_commons/URKeyMessage.h"
#include <ros/ros.h>

#include "ur_modern_driver/ur/consumer.h"

using namespace ros;

class KeyPublisher : public URMessagePacketConsumer
{
private:
  NodeHandle nh_;
  Publisher key_message_pub_;

  void publishKeyMessage(const KeyMessage& data) const;
public:
  KeyPublisher()
    : key_message_pub_(nh_.advertise<emma_commons::URKeyMessage>("ur_driver/key_message", 1))
  {
  }

  bool consume(VersionMessage& vm) { return true; }
  bool consume(KeyMessage& data);

  void setupConsumer()
  {
  }
  void teardownConsumer()
  {
  }
  void stopConsumer()
  {
  }
};
