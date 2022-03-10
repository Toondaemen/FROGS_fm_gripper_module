#ifndef _ROS_robotiq_ft_sensor_ft_sensor_h
#define _ROS_robotiq_ft_sensor_ft_sensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotiq_ft_sensor
{

  class ft_sensor : public ros::Msg
  {
    public:
      typedef float _Fx_type;
      _Fx_type Fx;
      typedef float _Fy_type;
      _Fy_type Fy;
      typedef float _Fz_type;
      _Fz_type Fz;
      typedef float _Mx_type;
      _Mx_type Mx;
      typedef float _My_type;
      _My_type My;
      typedef float _Mz_type;
      _Mz_type Mz;

    ft_sensor():
      Fx(0),
      Fy(0),
      Fz(0),
      Mx(0),
      My(0),
      Mz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_Fx;
      u_Fx.real = this->Fx;
      *(outbuffer + offset + 0) = (u_Fx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fx);
      union {
        float real;
        uint32_t base;
      } u_Fy;
      u_Fy.real = this->Fy;
      *(outbuffer + offset + 0) = (u_Fy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fy);
      union {
        float real;
        uint32_t base;
      } u_Fz;
      u_Fz.real = this->Fz;
      *(outbuffer + offset + 0) = (u_Fz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fz);
      union {
        float real;
        uint32_t base;
      } u_Mx;
      u_Mx.real = this->Mx;
      *(outbuffer + offset + 0) = (u_Mx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Mx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Mx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Mx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Mx);
      union {
        float real;
        uint32_t base;
      } u_My;
      u_My.real = this->My;
      *(outbuffer + offset + 0) = (u_My.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_My.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_My.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_My.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->My);
      union {
        float real;
        uint32_t base;
      } u_Mz;
      u_Mz.real = this->Mz;
      *(outbuffer + offset + 0) = (u_Mz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Mz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Mz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Mz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Mz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_Fx;
      u_Fx.base = 0;
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fx = u_Fx.real;
      offset += sizeof(this->Fx);
      union {
        float real;
        uint32_t base;
      } u_Fy;
      u_Fy.base = 0;
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fy = u_Fy.real;
      offset += sizeof(this->Fy);
      union {
        float real;
        uint32_t base;
      } u_Fz;
      u_Fz.base = 0;
      u_Fz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fz = u_Fz.real;
      offset += sizeof(this->Fz);
      union {
        float real;
        uint32_t base;
      } u_Mx;
      u_Mx.base = 0;
      u_Mx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Mx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Mx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Mx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Mx = u_Mx.real;
      offset += sizeof(this->Mx);
      union {
        float real;
        uint32_t base;
      } u_My;
      u_My.base = 0;
      u_My.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_My.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_My.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_My.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->My = u_My.real;
      offset += sizeof(this->My);
      union {
        float real;
        uint32_t base;
      } u_Mz;
      u_Mz.base = 0;
      u_Mz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Mz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Mz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Mz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Mz = u_Mz.real;
      offset += sizeof(this->Mz);
     return offset;
    }

    virtual const char * getType() override { return "robotiq_ft_sensor/ft_sensor"; };
    virtual const char * getMD5() override { return "060fd5fcbaefcff9daec9d5bb2d41714"; };

  };

}
#endif
