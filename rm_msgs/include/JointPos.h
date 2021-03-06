// Generated by gencpp from file rm_msgs/JointPos.msg
// DO NOT EDIT!


#ifndef RM_MSGS_MESSAGE_JOINTPOS_H
#define RM_MSGS_MESSAGE_JOINTPOS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rm_msgs
{
template <class ContainerAllocator>
struct JointPos_
{
  typedef JointPos_<ContainerAllocator> Type;

  JointPos_()
    : joint()  {
      joint.assign(0.0);
  }
  JointPos_(const ContainerAllocator& _alloc)
    : joint()  {
  (void)_alloc;
      joint.assign(0.0);
  }



   typedef boost::array<float, 6>  _joint_type;
  _joint_type joint;





  typedef boost::shared_ptr< ::rm_msgs::JointPos_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rm_msgs::JointPos_<ContainerAllocator> const> ConstPtr;

}; // struct JointPos_

typedef ::rm_msgs::JointPos_<std::allocator<void> > JointPos;

typedef boost::shared_ptr< ::rm_msgs::JointPos > JointPosPtr;
typedef boost::shared_ptr< ::rm_msgs::JointPos const> JointPosConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rm_msgs::JointPos_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rm_msgs::JointPos_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rm_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'rm_msgs': ['/home/nvidia/catkin_ws/src/RM_Robot/rm_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rm_msgs::JointPos_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rm_msgs::JointPos_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rm_msgs::JointPos_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rm_msgs::JointPos_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rm_msgs::JointPos_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rm_msgs::JointPos_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rm_msgs::JointPos_<ContainerAllocator> >
{
  static const char* value()
  {
    return "54f79eff4196767e5b029883dc8e8401";
  }

  static const char* value(const ::rm_msgs::JointPos_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x54f79eff4196767eULL;
  static const uint64_t static_value2 = 0x5b029883dc8e8401ULL;
};

template<class ContainerAllocator>
struct DataType< ::rm_msgs::JointPos_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rm_msgs/JointPos";
  }

  static const char* value(const ::rm_msgs::JointPos_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rm_msgs::JointPos_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#control arm joints without planning\n\
\n\
float32[6] joint\n\
";
  }

  static const char* value(const ::rm_msgs::JointPos_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rm_msgs::JointPos_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointPos_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rm_msgs::JointPos_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rm_msgs::JointPos_<ContainerAllocator>& v)
  {
    s << indent << "joint[]" << std::endl;
    for (size_t i = 0; i < v.joint.size(); ++i)
    {
      s << indent << "  joint[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.joint[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RM_MSGS_MESSAGE_JOINTPOS_H
