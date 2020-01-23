// Generated by gencpp from file robot_sim/pos_taskRequest.msg
// DO NOT EDIT!


#ifndef ROBOT_SIM_MESSAGE_POS_TASKREQUEST_H
#define ROBOT_SIM_MESSAGE_POS_TASKREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose2D.h>

namespace robot_sim
{
template <class ContainerAllocator>
struct pos_taskRequest_
{
  typedef pos_taskRequest_<ContainerAllocator> Type;

  pos_taskRequest_()
    : task_data()
    , task_type()
    , task_id(0)  {
    }
  pos_taskRequest_(const ContainerAllocator& _alloc)
    : task_data(_alloc)
    , task_type(_alloc)
    , task_id(0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _task_data_type;
  _task_data_type task_data;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _task_type_type;
  _task_type_type task_type;

   typedef int64_t _task_id_type;
  _task_id_type task_id;





  typedef boost::shared_ptr< ::robot_sim::pos_taskRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_sim::pos_taskRequest_<ContainerAllocator> const> ConstPtr;

}; // struct pos_taskRequest_

typedef ::robot_sim::pos_taskRequest_<std::allocator<void> > pos_taskRequest;

typedef boost::shared_ptr< ::robot_sim::pos_taskRequest > pos_taskRequestPtr;
typedef boost::shared_ptr< ::robot_sim::pos_taskRequest const> pos_taskRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_sim::pos_taskRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_sim::pos_taskRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robot_sim

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'robot_sim': ['/home/hcappel1/Multi_Agent_Systems/src/robot_sim/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robot_sim::pos_taskRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_sim::pos_taskRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_sim::pos_taskRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_sim::pos_taskRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_sim::pos_taskRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_sim::pos_taskRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_sim::pos_taskRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1998cd4f5d6808303435266496f15d7e";
  }

  static const char* value(const ::robot_sim::pos_taskRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1998cd4f5d680830ULL;
  static const uint64_t static_value2 = 0x3435266496f15d7eULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_sim::pos_taskRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_sim/pos_taskRequest";
  }

  static const char* value(const ::robot_sim::pos_taskRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_sim::pos_taskRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Pose2D task_data\n\
string task_type\n\
int64 task_id\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# Deprecated\n\
# Please use the full 3D pose.\n\
\n\
# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n\
\n\
# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n\
\n\
\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const ::robot_sim::pos_taskRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_sim::pos_taskRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.task_data);
      stream.next(m.task_type);
      stream.next(m.task_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pos_taskRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_sim::pos_taskRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_sim::pos_taskRequest_<ContainerAllocator>& v)
  {
    s << indent << "task_data: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.task_data);
    s << indent << "task_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.task_type);
    s << indent << "task_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.task_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_SIM_MESSAGE_POS_TASKREQUEST_H
