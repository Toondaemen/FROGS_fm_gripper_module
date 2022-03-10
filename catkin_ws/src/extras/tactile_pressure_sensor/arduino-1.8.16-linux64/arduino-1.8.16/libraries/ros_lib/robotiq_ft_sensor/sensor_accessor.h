#ifndef _ROS_SERVICE_sensor_accessor_h
#define _ROS_SERVICE_sensor_accessor_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotiq_ft_sensor
{

static const char SENSOR_ACCESSOR[] = "robotiq_ft_sensor/sensor_accessor";

  class sensor_accessorRequest : public ros::Msg
  {
    public:
      typedef uint8_t _command_id_type;
      _command_id_type command_id;
      typedef const char* _command_type;
      _command_type command;
      enum { COMMAND_GET_SERIAL_NUMBER = 1 };
      enum { COMMAND_GET_FIRMWARE_VERSION = 2 };
      enum { COMMAND_GET_PRODUCTION_YEAR = 4 };
      enum { COMMAND_SET_ZERO = 8 };

    sensor_accessorRequest():
      command_id(0),
      command("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->command_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->command_id);
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->command_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->command_id);
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
     return offset;
    }

    virtual const char * getType() override { return SENSOR_ACCESSOR; };
    virtual const char * getMD5() override { return "863e7f766b688715a9cf50b105733d81"; };

  };

  class sensor_accessorResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _res_type;
      _res_type res;

    sensor_accessorResponse():
      success(0),
      res("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_res = strlen(this->res);
      varToArr(outbuffer + offset, length_res);
      offset += 4;
      memcpy(outbuffer + offset, this->res, length_res);
      offset += length_res;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_res;
      arrToVar(length_res, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_res; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_res-1]=0;
      this->res = (char *)(inbuffer + offset-1);
      offset += length_res;
     return offset;
    }

    virtual const char * getType() override { return SENSOR_ACCESSOR; };
    virtual const char * getMD5() override { return "2adcefa00ba94fe7b359ee9018245fbf"; };

  };

  class sensor_accessor {
    public:
    typedef sensor_accessorRequest Request;
    typedef sensor_accessorResponse Response;
  };

}
#endif
