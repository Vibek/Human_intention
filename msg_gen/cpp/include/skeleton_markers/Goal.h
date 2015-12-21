/* Auto-generated by genmsg_cpp for file /home/vibek/skeleton_markers/msg/Goal.msg */
#ifndef SKELETON_MARKERS_MESSAGE_GOAL_H
#define SKELETON_MARKERS_MESSAGE_GOAL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace skeleton_markers
{
template <class ContainerAllocator>
struct Goal_ {
  typedef Goal_<ContainerAllocator> Type;

  Goal_()
  : name()
  {
  }

  Goal_(const ContainerAllocator& _alloc)
  : name(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;


  typedef boost::shared_ptr< ::skeleton_markers::Goal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::skeleton_markers::Goal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Goal
typedef  ::skeleton_markers::Goal_<std::allocator<void> > Goal;

typedef boost::shared_ptr< ::skeleton_markers::Goal> GoalPtr;
typedef boost::shared_ptr< ::skeleton_markers::Goal const> GoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::skeleton_markers::Goal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::skeleton_markers::Goal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace skeleton_markers

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::skeleton_markers::Goal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::skeleton_markers::Goal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::skeleton_markers::Goal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c1f3d28f1b044c871e6eff2e9fc3c667";
  }

  static const char* value(const  ::skeleton_markers::Goal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc1f3d28f1b044c87ULL;
  static const uint64_t static_value2 = 0x1e6eff2e9fc3c667ULL;
};

template<class ContainerAllocator>
struct DataType< ::skeleton_markers::Goal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "skeleton_markers/Goal";
  }

  static const char* value(const  ::skeleton_markers::Goal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::skeleton_markers::Goal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
\n\
";
  }

  static const char* value(const  ::skeleton_markers::Goal_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::skeleton_markers::Goal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Goal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::skeleton_markers::Goal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::skeleton_markers::Goal_<ContainerAllocator> & v) 
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SKELETON_MARKERS_MESSAGE_GOAL_H

