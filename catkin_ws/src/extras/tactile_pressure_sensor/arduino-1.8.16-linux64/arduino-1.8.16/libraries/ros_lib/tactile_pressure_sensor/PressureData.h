#ifndef _ROS_tactile_pressure_sensor_PressureData_h
#define _ROS_tactile_pressure_sensor_PressureData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float64MultiArray.h"

namespace tactile_pressure_sensor
{

  class PressureData : public ros::Msg
  {
    public:
      typedef const char* _sensor_state_type;
      _sensor_state_type sensor_state;
      typedef bool _slip_status_type;
      _slip_status_type slip_status;
      typedef float _slip_speed_type;
      _slip_speed_type slip_speed;
      typedef float _slip_angle_type;
      _slip_angle_type slip_angle;
      typedef float _estimated_force_type;
      _estimated_force_type estimated_force;
      typedef std_msgs::Float64MultiArray _estimated_location_type;
      _estimated_location_type estimated_location;

    PressureData():
      sensor_state(""),
      slip_status(0),
      slip_speed(0),
      slip_angle(0),
      estimated_force(0),
      estimated_location()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_sensor_state = strlen(this->sensor_state);
      varToArr(outbuffer + offset, length_sensor_state);
      offset += 4;
      memcpy(outbuffer + offset, this->sensor_state, length_sensor_state);
      offset += length_sensor_state;
      union {
        bool real;
        uint8_t base;
      } u_slip_status;
      u_slip_status.real = this->slip_status;
      *(outbuffer + offset + 0) = (u_slip_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->slip_status);
      offset += serializeAvrFloat64(outbuffer + offset, this->slip_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->slip_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->estimated_force);
      offset += this->estimated_location.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_sensor_state;
      arrToVar(length_sensor_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sensor_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sensor_state-1]=0;
      this->sensor_state = (char *)(inbuffer + offset-1);
      offset += length_sensor_state;
      union {
        bool real;
        uint8_t base;
      } u_slip_status;
      u_slip_status.base = 0;
      u_slip_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->slip_status = u_slip_status.real;
      offset += sizeof(this->slip_status);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->slip_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->slip_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->estimated_force));
      offset += this->estimated_location.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "tactile_pressure_sensor/PressureData"; };
    virtual const char * getMD5() override { return "2657d6b4d2f2fd2804159f8379352421"; };

  };

}
#endif
