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

class Path {
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
    // Serializes a message object of type Path
    // Serialize message field [path_]
    bufferOffset = _arraySerializer.int32(obj.path_, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Path
    let len;
    let data = new Path(null);
    // Deserialize message field [path_]
    data.path_ = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.path_.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'st_pid/Path';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4482bbc958240ef8af7049ce9e00f0e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] path_
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Path(null);
    if (msg.path_ !== undefined) {
      resolved.path_ = msg.path_;
    }
    else {
      resolved.path_ = []
    }

    return resolved;
    }
};

module.exports = Path;
