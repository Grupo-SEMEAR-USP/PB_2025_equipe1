// Auto-generated. Do not edit!

// (in-package robot_communication.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class velocity_comm {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_vel = null;
      this.right_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('left_vel')) {
        this.left_vel = initObj.left_vel
      }
      else {
        this.left_vel = 0.0;
      }
      if (initObj.hasOwnProperty('right_vel')) {
        this.right_vel = initObj.right_vel
      }
      else {
        this.right_vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type velocity_comm
    // Serialize message field [left_vel]
    bufferOffset = _serializer.float64(obj.left_vel, buffer, bufferOffset);
    // Serialize message field [right_vel]
    bufferOffset = _serializer.float64(obj.right_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type velocity_comm
    let len;
    let data = new velocity_comm(null);
    // Deserialize message field [left_vel]
    data.left_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_vel]
    data.right_vel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_communication/velocity_comm';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6276768e5df1d6b78b96e63931fb03fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # velocity_comm.msg
    float64 left_vel
    float64 right_vel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new velocity_comm(null);
    if (msg.left_vel !== undefined) {
      resolved.left_vel = msg.left_vel;
    }
    else {
      resolved.left_vel = 0.0
    }

    if (msg.right_vel !== undefined) {
      resolved.right_vel = msg.right_vel;
    }
    else {
      resolved.right_vel = 0.0
    }

    return resolved;
    }
};

module.exports = velocity_comm;
