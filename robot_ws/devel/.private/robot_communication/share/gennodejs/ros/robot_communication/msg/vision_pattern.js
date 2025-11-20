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

class vision_pattern {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.curvature = null;
      this.offset = null;
    }
    else {
      if (initObj.hasOwnProperty('curvature')) {
        this.curvature = initObj.curvature
      }
      else {
        this.curvature = 0.0;
      }
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vision_pattern
    // Serialize message field [curvature]
    bufferOffset = _serializer.float32(obj.curvature, buffer, bufferOffset);
    // Serialize message field [offset]
    bufferOffset = _serializer.float32(obj.offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vision_pattern
    let len;
    let data = new vision_pattern(null);
    // Deserialize message field [curvature]
    data.curvature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [offset]
    data.offset = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_communication/vision_pattern';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f7fe8c9b73d40487b6bbeb43e558374';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # vision_pattern.msg
    float32 curvature
    float32 offset
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vision_pattern(null);
    if (msg.curvature !== undefined) {
      resolved.curvature = msg.curvature;
    }
    else {
      resolved.curvature = 0.0
    }

    if (msg.offset !== undefined) {
      resolved.offset = msg.offset;
    }
    else {
      resolved.offset = 0.0
    }

    return resolved;
    }
};

module.exports = vision_pattern;
