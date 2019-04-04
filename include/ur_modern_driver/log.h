#pragma once
#include <inttypes.h>

#ifdef ROS_BUILD
#include <ros/ros.h>
#include "emma_commons/log.h"

#define LOG_DEBUG(format...) \
{\
  char info[512]; \
  sprintf(info, format);\
  emma_commons::Log::EMMA_INFO(__FILE__, __LINE__, info, "ur_driver", true);\
}
#define LOG_ERROR(format...)\
{\
  char err[512];\
  sprintf(err, format);\
  emma_commons::Log::EMMA_ERROR(__FILE__, __LINE__, err, "ur_driver");\
}
#define LOG_INFO(format...) \
{\
  char info[512]; \
  sprintf(info, format);\
  emma_commons::Log::EMMA_INFO(__FILE__, __LINE__, info, "ur_driver", true);\
}
#define LOG_WARN(format...) \
{\
  char err[512]; \
  sprintf(err, format);\
  emma_commons::Log::EMMA_ERROR(__FILE__, __LINE__, err, "ur_driver");\
}
#define LOG_FATAL(format...) \
{\
  char err[512]; \
  sprintf(err, format);\
  emma_commons::Log::EMMA_ERROR(__FILE__, __LINE__, err, "ur_driver");\
}

#else

#define LOG_DEBUG(format, ...) printf("[DEBUG]: " format "\n", ##__VA_ARGS__)
#define LOG_WARN(format, ...) printf("[WARNING]: " format "\n", ##__VA_ARGS__)
#define LOG_INFO(format, ...) printf("[INFO]: " format "\n", ##__VA_ARGS__)
#define LOG_ERROR(format, ...) printf("[ERROR]: " format "\n", ##__VA_ARGS__)
#define LOG_FATAL(format, ...) printf("[FATAL]: " format "\n", ##__VA_ARGS__)

#endif