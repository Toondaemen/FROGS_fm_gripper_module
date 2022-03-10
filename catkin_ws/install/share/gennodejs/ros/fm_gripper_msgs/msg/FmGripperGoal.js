// Auto-generated. Do not edit!

// (in-package fm_gripper_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FmGripperGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.distance = null;
      this.velocity = null;
      this.force = null;
      this.calibrate = null;
      this.reset = null;
    }
    else {
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0;
      }
      if (initObj.hasOwnProperty('force')) {
        this.force = initObj.force
      }
      else {
        this.force = 0;
      }
      if (initObj.hasOwnProperty('calibrate')) {
        this.calibrate = initObj.calibrate
      }
      else {
        this.calibrate = false;
      }
      if (initObj.hasOwnProperty('reset')) {
        this.reset = initObj.reset
      }
      else {
        this.reset = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FmGripperGoal
    // Serialize message field [distance]
    bufferOffset = _serializer.int64(obj.distance, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.int64(obj.velocity, buffer, bufferOffset);
    // Serialize message field [force]
    bufferOffset = _serializer.int64(obj.force, buffer, bufferOffset);
    // Serialize message field [calibrate]
    bufferOffset = _serializer.bool(obj.calibrate, buffer, bufferOffset);
    // Serialize message field [reset]
    bufferOffset = _serializer.bool(obj.reset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FmGripperGoal
    let len;
    let data = new FmGripperGoal(null);
    // Deserialize message field [distance]
    data.distance = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [force]
    data.force = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [calibrate]
    data.calibrate = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reset]
    data.reset = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fm_gripper_msgs/FmGripperGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fff58c2b7a2066e0d202de176f11f4bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #goal         
    int64 distance
    int64 velocity
    int64 force
    bool calibrate
    bool reset
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FmGripperGoal(null);
    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0
    }

    if (msg.force !== undefined) {
      resolved.force = msg.force;
    }
    else {
      resolved.force = 0
    }

    if (msg.calibrate !== undefined) {
      resolved.calibrate = msg.calibrate;
    }
    else {
      resolved.calibrate = false
    }

    if (msg.reset !== undefined) {
      resolved.reset = msg.reset;
    }
    else {
      resolved.reset = false
    }

    return resolved;
    }
};

module.exports = FmGripperGoal;
