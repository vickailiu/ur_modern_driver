#pragma once

#include <cstdlib>
#include "ur_modern_driver/ur/consumer.h"
#include "ur_modern_driver/ur/messages_parser.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/ur/rt_parser.h"
#include "ur_modern_driver/ur/state_parser.h"
#include "ur_modern_driver/ur/stream.h"

#include "emma_commons/URKeyMessage.h"

static const int UR_PRIMARY_PORT = 30001;

class URFactory : private URMessagePacketConsumer
{
private:
  NodeHandle nh_;
  Publisher key_message_pub_;

  URStream stream_;
  URMessageParser parser_;

  uint8_t major_version_;
  uint8_t minor_version_;

  bool consume(VersionMessage& vm)
  {
    LOG_INFO("Got VersionMessage:");
    LOG_INFO("project name: %d", vm.project_name.c_str());
    LOG_INFO("version: %u.%u.%d", vm.major_version, vm.minor_version, vm.svn_version);
    LOG_INFO("build date: %s", vm.build_date.c_str());

    major_version_ = vm.major_version;
    minor_version_ = vm.minor_version;

    return true;
  }

  bool consume(KeyMessage& km)
  {
    LOG_INFO("Got KeyMessage:");
    LOG_INFO("robot message code: %d", km.robot_message_code);
    LOG_INFO("robot message argument: %d", km.robot_message_argument);
    LOG_INFO("message title: %s", km.message_title.c_str());
    LOG_INFO("text message: %s", km.text_message.c_str());

    // key_message_pub_.publish

    return true;
  }

  void setupConsumer()
  {
  }
  void teardownConsumer()
  {
  }
  void stopConsumer()
  {
  }

public:
  URFactory(std::string& host)
  : stream_(host, UR_PRIMARY_PORT)
  , key_message_pub_(nh_.advertise<emma_commons::URKeyMessage>("ur_driver/key_message", 1))
  {
    URProducer<MessagePacket> prod(stream_, parser_);
    std::vector<unique_ptr<MessagePacket>> results;

    prod.setupProducer();

    if (!prod.tryGet(results) || results.size() == 0)
    {
      LOG_FATAL("No version message received, init failed!");
      std::exit(EXIT_FAILURE);
    }

    for (auto const& p : results)
    {
      p->consumeWith(*this);
    }

    if (major_version_ == 0 && minor_version_ == 0)
    {
      LOG_FATAL("No version message received, init failed!");
      std::exit(EXIT_FAILURE);
    }

    prod.teardownProducer();
  }

  bool isVersion3()
  {
    return major_version_ == 3;
  }

  std::unique_ptr<URCommander> getCommander(URStream& stream)
  {
    if (major_version_ == 1)
      return std::unique_ptr<URCommander>(new URCommander_V1_X(stream));
    else if (minor_version_ < 3)
      return std::unique_ptr<URCommander>(new URCommander_V3_1__2(stream));
    else
      return std::unique_ptr<URCommander>(new URCommander_V3_3(stream));
  }

  std::unique_ptr<URParser<StatePacket>> getStateParser()
  {
    if (major_version_ == 1)
    {
      return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V1_X);
    }
    else
    {
      if (minor_version_ < 3)
        return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V3_0__1);
      else if (minor_version_ < 5)
        return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V3_2);
      else
        return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V3_5);
    }
  }

  std::unique_ptr<URParser<RTPacket>> getRTParser()
  {
    if (major_version_ == 1)
    {
      if (minor_version_ < 8)
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V1_6__7);
      else
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V1_8);
    }
    else
    {
      if (minor_version_ < 3)
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V3_0__1);
      else
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V3_2__3);
    }
  }
};
