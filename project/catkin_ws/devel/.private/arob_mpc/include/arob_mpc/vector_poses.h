// Generated by gencpp from file arob_mpc/vector_poses.msg
// DO NOT EDIT!


#ifndef AROB_MPC_MESSAGE_VECTOR_POSES_H
#define AROB_MPC_MESSAGE_VECTOR_POSES_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>

namespace arob_mpc
{
template <class ContainerAllocator>
struct vector_poses_
{
  typedef vector_poses_<ContainerAllocator> Type;

  vector_poses_()
    : poses()  {
    }
  vector_poses_(const ContainerAllocator& _alloc)
    : poses(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::PoseStamped_<ContainerAllocator> >> _poses_type;
  _poses_type poses;





  typedef boost::shared_ptr< ::arob_mpc::vector_poses_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arob_mpc::vector_poses_<ContainerAllocator> const> ConstPtr;

}; // struct vector_poses_

typedef ::arob_mpc::vector_poses_<std::allocator<void> > vector_poses;

typedef boost::shared_ptr< ::arob_mpc::vector_poses > vector_posesPtr;
typedef boost::shared_ptr< ::arob_mpc::vector_poses const> vector_posesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::arob_mpc::vector_poses_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::arob_mpc::vector_poses_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::arob_mpc::vector_poses_<ContainerAllocator1> & lhs, const ::arob_mpc::vector_poses_<ContainerAllocator2> & rhs)
{
  return lhs.poses == rhs.poses;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::arob_mpc::vector_poses_<ContainerAllocator1> & lhs, const ::arob_mpc::vector_poses_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace arob_mpc

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::arob_mpc::vector_poses_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::arob_mpc::vector_poses_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::arob_mpc::vector_poses_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::arob_mpc::vector_poses_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arob_mpc::vector_poses_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arob_mpc::vector_poses_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::arob_mpc::vector_poses_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f4bbbec105a3dc69d6c5974def547813";
  }

  static const char* value(const ::arob_mpc::vector_poses_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf4bbbec105a3dc69ULL;
  static const uint64_t static_value2 = 0xd6c5974def547813ULL;
};

template<class ContainerAllocator>
struct DataType< ::arob_mpc::vector_poses_<ContainerAllocator> >
{
  static const char* value()
  {
    return "arob_mpc/vector_poses";
  }

  static const char* value(const ::arob_mpc::vector_poses_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::arob_mpc::vector_poses_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/PoseStamped[] poses\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
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
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::arob_mpc::vector_poses_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::arob_mpc::vector_poses_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.poses);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct vector_poses_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arob_mpc::vector_poses_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::arob_mpc::vector_poses_<ContainerAllocator>& v)
  {
    s << indent << "poses[]" << std::endl;
    for (size_t i = 0; i < v.poses.size(); ++i)
    {
      s << indent << "  poses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.poses[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // AROB_MPC_MESSAGE_VECTOR_POSES_H