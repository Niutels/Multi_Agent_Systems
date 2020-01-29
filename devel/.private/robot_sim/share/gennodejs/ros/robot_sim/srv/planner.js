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

class plannerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.coords_msg = null;
    }
    else {
      if (initObj.hasOwnProperty('coords_msg')) {
        this.coords_msg = initObj.coords_msg
      }
      else {
        this.coords_msg = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type plannerRequest
    // Serialize message field [coords_msg]
    bufferOffset = _arraySerializer.float32(obj.coords_msg, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type plannerRequest
    let len;
    let data = new plannerRequest(null);
    // Deserialize message field [coords_msg]
    data.coords_msg = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.coords_msg.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_sim/plannerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa571bd3d8279217f2fb0b2475d614e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] coords_msg
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new plannerRequest(null);
    if (msg.coords_msg !== undefined) {
      resolved.coords_msg = msg.coords_msg;
    }
    else {
      resolved.coords_msg = []
    }

    return resolved;
    }
};

class plannerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.optimal_path = null;
    }
    else {
      if (initObj.hasOwnProperty('optimal_path')) {
        this.optimal_path = initObj.optimal_path
      }
      else {
        this.optimal_path = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type plannerResponse
    // Serialize message field [optimal_path]
    bufferOffset = _arraySerializer.float32(obj.optimal_path, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type plannerResponse
    let len;
    let data = new plannerResponse(null);
    // Deserialize message field [optimal_path]
    data.optimal_path = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.optimal_path.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_sim/plannerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '34aebd8b0a17f86944f7baf8bcfd2d93';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] optimal_path
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new plannerResponse(null);
    if (msg.optimal_path !== undefined) {
      resolved.optimal_path = msg.optimal_path;
    }
    else {
      resolved.optimal_path = []
    }

    return resolved;
    }
};

module.exports = {
  Request: plannerRequest,
  Response: plannerResponse,
  md5sum() { return '6423f791599f0a66e62771c89cda1dcd'; },
  datatype() { return 'robot_sim/planner'; }
};
