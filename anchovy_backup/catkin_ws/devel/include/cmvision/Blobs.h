// Generated by gencpp from file cmvision/Blobs.msg
// DO NOT EDIT!


#ifndef CMVISION_MESSAGE_BLOBS_H
#define CMVISION_MESSAGE_BLOBS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <cmvision/Blob.h>

namespace cmvision
{
template <class ContainerAllocator>
struct Blobs_
{
  typedef Blobs_<ContainerAllocator> Type;

  Blobs_()
    : header()
    , image_width(0)
    , image_height(0)
    , blob_count(0)
    , blobs()  {
    }
  Blobs_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , image_width(0)
    , image_height(0)
    , blob_count(0)
    , blobs(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _image_width_type;
  _image_width_type image_width;

   typedef uint32_t _image_height_type;
  _image_height_type image_height;

   typedef uint32_t _blob_count_type;
  _blob_count_type blob_count;

   typedef std::vector< ::cmvision::Blob_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::cmvision::Blob_<ContainerAllocator> >::other >  _blobs_type;
  _blobs_type blobs;




  typedef boost::shared_ptr< ::cmvision::Blobs_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cmvision::Blobs_<ContainerAllocator> const> ConstPtr;

}; // struct Blobs_

typedef ::cmvision::Blobs_<std::allocator<void> > Blobs;

typedef boost::shared_ptr< ::cmvision::Blobs > BlobsPtr;
typedef boost::shared_ptr< ::cmvision::Blobs const> BlobsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cmvision::Blobs_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cmvision::Blobs_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cmvision

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'cmvision': ['/home/anchovy/catkin_ws/src/cmvision/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cmvision::Blobs_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cmvision::Blobs_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cmvision::Blobs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cmvision::Blobs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cmvision::Blobs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cmvision::Blobs_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cmvision::Blobs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9095431d60142fc813f87d8cc9018af4";
  }

  static const char* value(const ::cmvision::Blobs_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9095431d60142fc8ULL;
  static const uint64_t static_value2 = 0x13f87d8cc9018af4ULL;
};

template<class ContainerAllocator>
struct DataType< ::cmvision::Blobs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cmvision/Blobs";
  }

  static const char* value(const ::cmvision::Blobs_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cmvision::Blobs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
uint32 image_width\n\
uint32 image_height\n\
uint32 blob_count\n\
Blob[] blobs\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: cmvision/Blob\n\
string name\n\
uint32 red\n\
uint32 green\n\
uint32 blue\n\
uint32 area\n\
uint32 x\n\
uint32 y\n\
uint32 left\n\
uint32 right\n\
uint32 top\n\
uint32 bottom\n\
";
  }

  static const char* value(const ::cmvision::Blobs_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cmvision::Blobs_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.image_width);
      stream.next(m.image_height);
      stream.next(m.blob_count);
      stream.next(m.blobs);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Blobs_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cmvision::Blobs_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cmvision::Blobs_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "image_width: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.image_width);
    s << indent << "image_height: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.image_height);
    s << indent << "blob_count: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.blob_count);
    s << indent << "blobs[]" << std::endl;
    for (size_t i = 0; i < v.blobs.size(); ++i)
    {
      s << indent << "  blobs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::cmvision::Blob_<ContainerAllocator> >::stream(s, indent + "    ", v.blobs[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CMVISION_MESSAGE_BLOBS_H
