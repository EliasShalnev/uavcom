#pragma once

#include <string>
#include <map>

namespace StreamTopic
{
using ReasonCode = uint8_t;

constexpr ReasonCode OK = 0x00;
constexpr ReasonCode IsStreaming = OK+1;
constexpr ReasonCode IsUnreachable = IsStreaming+1;

//maybe move to .cpp file
inline std::string codeExplanation(const ReasonCode code) 
{
    if(code == OK) { return std::string("Request for streaming was accepted."); }
    else if(code == IsStreaming) { return std::string("Requested topic is already streaming."); }
    else if(code == IsUnreachable) { return std::string("Destination is unreachable"); }
}

} //namespace StreamTopic