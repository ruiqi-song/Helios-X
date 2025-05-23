// Generated by gencpp from file waytous_msgs/ObjectArray.msg
// DO NOT EDIT!


#ifndef WAYTOUS_MSGS_MESSAGE_OBJECTARRAY_H
#define WAYTOUS_MSGS_MESSAGE_OBJECTARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include "message/rosmsgs/obstacles/Object.h"

namespace waytous_msgs
{
template <class ContainerAllocator>
struct ObjectArray_
{
  typedef ObjectArray_<ContainerAllocator> Type;

  ObjectArray_()
    : header()
    , object_array()  {
    }
  ObjectArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , object_array(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::waytous_msgs::Object_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::waytous_msgs::Object_<ContainerAllocator> >::other >  _object_array_type;
  _object_array_type object_array;





  typedef boost::shared_ptr< ::waytous_msgs::ObjectArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::waytous_msgs::ObjectArray_<ContainerAllocator> const> ConstPtr;

}; // struct ObjectArray_

typedef ::waytous_msgs::ObjectArray_<std::allocator<void> > ObjectArray;

typedef boost::shared_ptr< ::waytous_msgs::ObjectArray > ObjectArrayPtr;
typedef boost::shared_ptr< ::waytous_msgs::ObjectArray const> ObjectArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::waytous_msgs::ObjectArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::waytous_msgs::ObjectArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::waytous_msgs::ObjectArray_<ContainerAllocator1> & lhs, const ::waytous_msgs::ObjectArray_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.object_array == rhs.object_array;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::waytous_msgs::ObjectArray_<ContainerAllocator1> & lhs, const ::waytous_msgs::ObjectArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace waytous_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::waytous_msgs::ObjectArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::waytous_msgs::ObjectArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::waytous_msgs::ObjectArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::waytous_msgs::ObjectArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::waytous_msgs::ObjectArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::waytous_msgs::ObjectArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::waytous_msgs::ObjectArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7cfea5e4913d3130f1b6444caefffd7c";
  }

  static const char* value(const ::waytous_msgs::ObjectArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7cfea5e4913d3130ULL;
  static const uint64_t static_value2 = 0xf1b6444caefffd7cULL;
};

template<class ContainerAllocator>
struct DataType< ::waytous_msgs::ObjectArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "waytous_msgs/ObjectArray";
  }

  static const char* value(const ::waytous_msgs::ObjectArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::waytous_msgs::ObjectArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"Object[] object_array\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: waytous_msgs/Object\n"
"sensor_msgs/PointCloud2 cluster\n"
"sensor_msgs/RegionOfInterest roi\n"
"\n"
"uint8 ROCK=0\n"
"uint8 PERSON=1\n"
"uint8 LORRY=2\n"
"uint8 TRUCK=3\n"
"uint8 SIGN=4\n"
"uint8 CAR=5\n"
"uint8 AUXILIARY=6\n"
"uint8 WARNING=7\n"
"uint8 PUDDLE=8\n"
"uint8 EXCAVATOR=9\n"
"uint8 HOUSE=10\n"
"uint32 type\n"
"float64 confidence\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/PointCloud2\n"
"# This message holds a collection of N-dimensional points, which may\n"
"# contain additional information such as normals, intensity, etc. The\n"
"# point data is stored as a binary blob, its layout described by the\n"
"# contents of the \"fields\" array.\n"
"\n"
"# The point cloud data may be organized 2d (image-like) or 1d\n"
"# (unordered). Point clouds organized as 2d images may be produced by\n"
"# camera depth sensors such as stereo or time-of-flight.\n"
"\n"
"# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n"
"# points).\n"
"Header header\n"
"\n"
"# 2D structure of the point cloud. If the cloud is unordered, height is\n"
"# 1 and width is the length of the point cloud.\n"
"uint32 height\n"
"uint32 width\n"
"\n"
"# Describes the channels and their layout in the binary data blob.\n"
"PointField[] fields\n"
"\n"
"bool    is_bigendian # Is this data bigendian?\n"
"uint32  point_step   # Length of a point in bytes\n"
"uint32  row_step     # Length of a row in bytes\n"
"uint8[] data         # Actual point data, size is (row_step*height)\n"
"\n"
"bool is_dense        # True if there are no invalid points\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/PointField\n"
"# This message holds the description of one point entry in the\n"
"# PointCloud2 message format.\n"
"uint8 INT8    = 1\n"
"uint8 UINT8   = 2\n"
"uint8 INT16   = 3\n"
"uint8 UINT16  = 4\n"
"uint8 INT32   = 5\n"
"uint8 UINT32  = 6\n"
"uint8 FLOAT32 = 7\n"
"uint8 FLOAT64 = 8\n"
"\n"
"string name      # Name of field\n"
"uint32 offset    # Offset from start of point struct\n"
"uint8  datatype  # Datatype enumeration, see above\n"
"uint32 count     # How many elements in the field\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/RegionOfInterest\n"
"# This message is used to specify a region of interest within an image.\n"
"#\n"
"# When used to specify the ROI setting of the camera when the image was\n"
"# taken, the height and width fields should either match the height and\n"
"# width fields for the associated image; or height = width = 0\n"
"# indicates that the full resolution image was captured.\n"
"\n"
"uint32 x_offset  # Leftmost pixel of the ROI\n"
"                 # (0 if the ROI includes the left edge of the image)\n"
"uint32 y_offset  # Topmost pixel of the ROI\n"
"                 # (0 if the ROI includes the top edge of the image)\n"
"uint32 height    # Height of ROI\n"
"uint32 width     # Width of ROI\n"
"\n"
"# True if a distinct rectified ROI should be calculated from the \"raw\"\n"
"# ROI in this message. Typically this should be False if the full image\n"
"# is captured (ROI not used), and True if a subwindow is captured (ROI\n"
"# used).\n"
"bool do_rectify\n"
;
  }

  static const char* value(const ::waytous_msgs::ObjectArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::waytous_msgs::ObjectArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.object_array);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObjectArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::waytous_msgs::ObjectArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::waytous_msgs::ObjectArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "object_array[]" << std::endl;
    for (size_t i = 0; i < v.object_array.size(); ++i)
    {
      s << indent << "  object_array[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::waytous_msgs::Object_<ContainerAllocator> >::stream(s, indent + "    ", v.object_array[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // WAYTOUS_MSGS_MESSAGE_OBJECTARRAY_H
