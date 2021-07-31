#pragma once

#include <string>

namespace def{


using TopicName = std::string;
using BoardName = std::string;

/****Globals****/
const std::string g_uavNodeName = "uav_com";
const std::string g_input = "input";
const std::string g_output = "output";
const std::string g_cone = "cone_";
const std::string g_broadcast = "broadcast";
const std::string g_heartbeat = "heartbeat";


} //namespace def