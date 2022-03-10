// Generated by gencpp from file fm_gripper_msgs/FmGripperActionGoal.msg
// DO NOT EDIT!


#ifndef FM_GRIPPER_MSGS_MESSAGE_FMGRIPPERACTIONGOAL_H
#define FM_GRIPPER_MSGS_MESSAGE_FMGRIPPERACTIONGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalID.h>
#include <fm_gripper_msgs/FmGripperGoal.h>

namespace fm_gripper_msgs
{
template <class ContainerAllocator>
struct FmGripperActionGoal_
{
  typedef FmGripperActionGoal_<ContainerAllocator> Type;

  FmGripperActionGoal_()
    : header()
    , goal_id()
    , goal()  {
    }
  FmGripperActionGoal_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , goal_id(_alloc)
    , goal(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
  _goal_id_type goal_id;

   typedef  ::fm_gripper_msgs::FmGripperGoal_<ContainerAllocator>  _goal_type;
  _goal_type goal;





  typedef boost::shared_ptr< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> const> ConstPtr;

}; // struct FmGripperActionGoal_

typedef ::fm_gripper_msgs::FmGripperActionGoal_<std::allocator<void> > FmGripperActionGoal;

typedef boost::shared_ptr< ::fm_gripper_msgs::FmGripperActionGoal > FmGripperActionGoalPtr;
typedef boost::shared_ptr< ::fm_gripper_msgs::FmGripperActionGoal const> FmGripperActionGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator1> & lhs, const ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.goal_id == rhs.goal_id &&
    lhs.goal == rhs.goal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator1> & lhs, const ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace fm_gripper_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a3b6c7a3d60a4f98b9d3c514fdad29fd";
  }

  static const char* value(const ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa3b6c7a3d60a4f98ULL;
  static const uint64_t static_value2 = 0xb9d3c514fdad29fdULL;
};

template<class ContainerAllocator>
struct DataType< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fm_gripper_msgs/FmGripperActionGoal";
  }

  static const char* value(const ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"FmGripperGoal goal\n"
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
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: fm_gripper_msgs/FmGripperGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#goal         \n"
"int64 distance\n"
"int64 velocity\n"
"int64 force\n"
"bool calibrate\n"
"bool reset\n"
;
  }

  static const char* value(const ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.goal_id);
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FmGripperActionGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fm_gripper_msgs::FmGripperActionGoal_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
    s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::fm_gripper_msgs::FmGripperGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FM_GRIPPER_MSGS_MESSAGE_FMGRIPPERACTIONGOAL_H
