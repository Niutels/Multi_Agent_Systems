// Auto-generated. Do not edit!

// (in-package robot_sim.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class pos_taskRequest {
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
        this.task_data = new geometry_msgs.msg.Pose2D();
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
    // Serializes a message object of type pos_taskRequest
    // Serialize message field [task_data]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.task_data, buffer, bufferOffset);
    // Serialize message field [task_type]
    bufferOffset = _serializer.string(obj.task_type, buffer, bufferOffset);
    // Serialize message field [task_id]
    bufferOffset = _serializer.int64(obj.task_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pos_taskRequest
    let len;
    let data = new pos_taskRequest(null);
    // Deserialize message field [task_data]
    data.task_data = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [task_type]
    data.task_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [task_id]
    data.task_id = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.task_type.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_sim/pos_taskRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1998cd4f5d6808303435266496f15d7e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose2D task_data
    string task_type
    int64 task_id
    
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # Deprecated
    # Please use the full 3D pose.
    
    # In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.
    
    # If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.
    
    
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pos_taskRequest(null);
    if (msg.task_data !== undefined) {
      resolved.task_data = geometry_msgs.msg.Pose2D.Resolve(msg.task_data)
    }
    else {
      resolved.task_data = new geometry_msgs.msg.Pose2D()
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

class pos_taskResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pos_taskResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pos_taskResponse
    let len;
    let data = new pos_taskResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_sim/pos_taskResponse';
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
    const resolved = new pos_taskResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: pos_taskRequest,
  Response: pos_taskResponse,
  md5sum() { return '1998cd4f5d6808303435266496f15d7e'; },
  datatype() { return 'robot_sim/pos_task'; }
};
