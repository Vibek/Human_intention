/* Auto-generated by genmsg_cpp for file /home/vibek/skeleton_markers/msg/PerceptInfo.msg */
#ifndef SKELETON_MARKERS_MESSAGE_PERCEPTINFO_H
#define SKELETON_MARKERS_MESSAGE_PERCEPTINFO_H
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
struct PerceptInfo_ {
  typedef PerceptInfo_<ContainerAllocator> Type;

  PerceptInfo_()
  : class_id()
  , class_support(0.0)
  , object_id()
  , object_support(0.0)
  , name()
  , orientation()
  {
  }

  PerceptInfo_(const ContainerAllocator& _alloc)
  : class_id(_alloc)
  , class_support(0.0)
  , object_id(_alloc)
  , object_support(0.0)
  , name(_alloc)
  , orientation(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _class_id_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  class_id;

  typedef float _class_support_type;
  float class_support;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _object_id_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  object_id;

  typedef float _object_support_type;
  float object_support;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _orientation_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  orientation;


  typedef boost::shared_ptr< ::skeleton_markers::PerceptInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::skeleton_markers::PerceptInfo_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PerceptInfo
typedef  ::skeleton_markers::PerceptInfo_<std::allocator<void> > PerceptInfo;

typedef boost::shared_ptr< ::skeleton_markers::PerceptInfo> PerceptInfoPtr;
typedef boost::shared_ptr< ::skeleton_markers::PerceptInfo const> PerceptInfoConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::skeleton_markers::PerceptInfo_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::skeleton_markers::PerceptInfo_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace skeleton_markers

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::skeleton_markers::PerceptInfo_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::skeleton_markers::PerceptInfo_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::skeleton_markers::PerceptInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "712b3ec12b55ad51984514c0d645b54b";
  }

  static const char* value(const  ::skeleton_markers::PerceptInfo_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x712b3ec12b55ad51ULL;
  static const uint64_t static_value2 = 0x984514c0d645b54bULL;
};

template<class ContainerAllocator>
struct DataType< ::skeleton_markers::PerceptInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "skeleton_markers/PerceptInfo";
  }

  static const char* value(const  ::skeleton_markers::PerceptInfo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::skeleton_markers::PerceptInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string class_id\n\
float32 class_support\n\
string object_id\n\
float32 object_support\n\
string name\n\
string orientation\n\
\n\
";
  }

  static const char* value(const  ::skeleton_markers::PerceptInfo_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::skeleton_markers::PerceptInfo_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.class_id);
    stream.next(m.class_support);
    stream.next(m.object_id);
    stream.next(m.object_support);
    stream.next(m.name);
    stream.next(m.orientation);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PerceptInfo_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::skeleton_markers::PerceptInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::skeleton_markers::PerceptInfo_<ContainerAllocator> & v) 
  {
    s << indent << "class_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.class_id);
    s << indent << "class_support: ";
    Printer<float>::stream(s, indent + "  ", v.class_support);
    s << indent << "object_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.object_id);
    s << indent << "object_support: ";
    Printer<float>::stream(s, indent + "  ", v.object_support);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "orientation: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.orientation);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SKELETON_MARKERS_MESSAGE_PERCEPTINFO_H

