// Generated by gencpp from file robot_sim/other_task.msg
// DO NOT EDIT!


#ifndef ROBOT_SIM_MESSAGE_OTHER_TASK_H
#define ROBOT_SIM_MESSAGE_OTHER_TASK_H

#include <ros/service_traits.h>


#include <robot_sim/other_taskRequest.h>
#include <robot_sim/other_taskResponse.h>


namespace robot_sim
{

struct other_task
{

typedef other_taskRequest Request;
typedef other_taskResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct other_task
} // namespace robot_sim


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::robot_sim::other_task > {
  static const char* value()
  {
    return "d0f539cb3e5e23bc53002dc7b1f24555";
  }

  static const char* value(const ::robot_sim::other_task&) { return value(); }
};

template<>
struct DataType< ::robot_sim::other_task > {
  static const char* value()
  {
    return "robot_sim/other_task";
  }

  static const char* value(const ::robot_sim::other_task&) { return value(); }
};


// service_traits::MD5Sum< ::robot_sim::other_taskRequest> should match 
// service_traits::MD5Sum< ::robot_sim::other_task > 
template<>
struct MD5Sum< ::robot_sim::other_taskRequest>
{
  static const char* value()
  {
    return MD5Sum< ::robot_sim::other_task >::value();
  }
  static const char* value(const ::robot_sim::other_taskRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::robot_sim::other_taskRequest> should match 
// service_traits::DataType< ::robot_sim::other_task > 
template<>
struct DataType< ::robot_sim::other_taskRequest>
{
  static const char* value()
  {
    return DataType< ::robot_sim::other_task >::value();
  }
  static const char* value(const ::robot_sim::other_taskRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::robot_sim::other_taskResponse> should match 
// service_traits::MD5Sum< ::robot_sim::other_task > 
template<>
struct MD5Sum< ::robot_sim::other_taskResponse>
{
  static const char* value()
  {
    return MD5Sum< ::robot_sim::other_task >::value();
  }
  static const char* value(const ::robot_sim::other_taskResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::robot_sim::other_taskResponse> should match 
// service_traits::DataType< ::robot_sim::other_task > 
template<>
struct DataType< ::robot_sim::other_taskResponse>
{
  static const char* value()
  {
    return DataType< ::robot_sim::other_task >::value();
  }
  static const char* value(const ::robot_sim::other_taskResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROBOT_SIM_MESSAGE_OTHER_TASK_H
