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
    LOG_INFO("sendDashboardCmd called");
    return (resp.result = commander_.sendDashboardCmd(req.cmd));
  }

public:
  DashboardService(URCommander& commander)
    : commander_(commander)
    , dashboard_service_(nh_.advertiseService("ur_driver/dashboard_cmd", &DashboardService::sendDashboardCmd, this))
  {
  }
};
