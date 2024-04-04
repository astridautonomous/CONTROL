// Auto-generated. Do not edit!

// (in-package st_pid.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Pat {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path_ = null;
    }
    else {
      if (initObj.hasOwnProperty('path_')) {
        this.path_ = initObj.path_
      }
      else {
        this.path_ = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Pat
    // Serialize message field [path_]
    bufferOffset = _arraySerializer.uint8(obj.path_, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Pat
    let len;
    let data = new Pat(null);
    // Deserialize message field [path_]
    data.path_ = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.path_.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'st_pid/Pat';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e7e4cb758a1a3e10b6dab1976b351ef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[] path_
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Pat(null);
    if (msg.path_ !== undefined) {
      resolved.path_ = msg.path_;
    }
    else {
      resolved.path_ = []
    }

    return resolved;
    }
};

module.exports = Pat;
