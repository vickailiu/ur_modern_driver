#pragma once

#include <ros/ros.h>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/commander.h"

#include "emma_commons/URDashboard.h"

class DashboardService
{
private:
  ros::NodeHandle nh_;
  URCommander& commander_;
  ros::ServiceServer dashboard_service_;

  bool sendDashboardCmd(emma_commons::URDashboardRequest& req, emma_commons::URDashboardResponse& resp)
  {
    addrinfo info;

    memset(&info, 0, sizeof(info));
    info.ai_family = AF_UNSPEC;
    info.ai_socktype = SOCK_STREAM;

    addrinfo* res = 0;

    if(getaddrinfo("192.168.0.49", "29999", &info, &res) != 0 || !res)
    {
      perror("Could not get address for UR host");
      return false;
    }

    sockaddr_storage addr;
    socklen_t addrlen = res->ai_addrlen;
    memcpy(&addr, res->ai_addr, res->ai_addrlen);

    int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);

    freeaddrinfo(res);

    if(fd < 0)
    {
      perror("socket");
      return false;
    }

    if(connect(fd, (const sockaddr*)&addr, addrlen) != 0)
    {
      perror("Could not connect to UR dashboard");
      return false;
    }

    if(write(fd, req.cmd.c_str(), strlen(req.cmd.c_str())) != strlen(req.cmd.c_str()))
    {
      perror("Could not write to UR dashboard");
      return false;
    }

    close(fd);

//    LOG_INFO("sendDashboardCmd called");
//    return (resp.result = commander_.sendDashboardCmd(req.cmd));
  }

public:
  DashboardService(URCommander& commander)
    : commander_(commander)
    , dashboard_service_(nh_.advertiseService("ur_driver/dashboard_cmd", &DashboardService::sendDashboardCmd, this))
  {
  }
};
