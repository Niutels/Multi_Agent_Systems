// Auto-generated. Do not edit!

// (in-package robot_sim.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class other_taskRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task_data = null;
      this.task_type = null;
      this.task_id = null;
    }
    else {
      if (initObj.hasOwnProperty('task_data')) {
        this.task_data = initObj.task_data
      }
      else {
        this.task_data = 0;
      }
      if (initObj.hasOwnProperty('task_type')) {
        this.task_type = initObj.task_type
      }
      else {
        this.task_type = '';
      }
      if (initObj.hasOwnProperty('task_id')) {
        this.task_id = initObj.task_id
      }
      else {
        this.task_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type other_taskRequest
    // Serialize message field [task_data]
    bufferOffset = _serializer.int32(obj.task_data, buffer, bufferOffset);
    // Serialize message field [task_type]
    bufferOffset = _serializer.string(obj.task_type, buffer, bufferOffset);
    // Serialize message field [task_id]
    bufferOffset = _serializer.int64(obj.task_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type other_taskRequest
    let len;
    let data = new other_taskRequest(null);
    // Deserialize message field [task_data]
    data.task_data = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [task_type]
    data.task_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [task_id]
    data.task_id = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.task_type.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_sim/other_taskRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd0f539cb3e5e23bc53002dc7b1f24555';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 task_data
    string task_type
    int64 task_id
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new other_taskRequest(null);
    if (msg.task_data !== undefined) {
      resolved.task_data = msg.task_data;
    }
    else {
      resolved.task_data = 0
    }

    if (msg.task_type !== undefined) {
      resolved.task_type = msg.task_type;
    }
    else {
      resolved.task_type = ''
    }

    if (msg.task_id !== undefined) {
      resolved.task_id = msg.task_id;
    }
    else {
      resolved.task_id = 0
    }

    return resolved;
    }
};

class other_taskResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type other_taskResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type other_taskResponse
    let len;
    let data = new other_taskResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_sim/other_taskResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new other_taskResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: other_taskRequest,
  Response: other_taskResponse,
  md5sum() { return 'd0f539cb3e5e23bc53002dc7b1f24555'; },
  datatype() { return 'robot_sim/other_task'; }
};
