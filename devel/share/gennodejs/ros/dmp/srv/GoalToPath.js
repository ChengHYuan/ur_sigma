// Auto-generated. Do not edit!

// (in-package dmp.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let nav_msgs = _finder('nav_msgs');

//-----------------------------------------------------------

class GoalToPathRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Start = null;
      this.GoalIndex = null;
    }
    else {
      if (initObj.hasOwnProperty('Start')) {
        this.Start = initObj.Start
      }
      else {
        this.Start = [];
      }
      if (initObj.hasOwnProperty('GoalIndex')) {
        this.GoalIndex = initObj.GoalIndex
      }
      else {
        this.GoalIndex = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoalToPathRequest
    // Serialize message field [Start]
    bufferOffset = _arraySerializer.float64(obj.Start, buffer, bufferOffset, null);
    // Serialize message field [GoalIndex]
    bufferOffset = _serializer.int32(obj.GoalIndex, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoalToPathRequest
    let len;
    let data = new GoalToPathRequest(null);
    // Deserialize message field [Start]
    data.Start = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [GoalIndex]
    data.GoalIndex = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.Start.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dmp/GoalToPathRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '47f8665a74074523c873649f357d3ba7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] Start
    
    int32 GoalIndex
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GoalToPathRequest(null);
    if (msg.Start !== undefined) {
      resolved.Start = msg.Start;
    }
    else {
      resolved.Start = []
    }

    if (msg.GoalIndex !== undefined) {
      resolved.GoalIndex = msg.GoalIndex;
    }
    else {
      resolved.GoalIndex = 0
    }

    return resolved;
    }
};

class GoalToPathResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Path = null;
    }
    else {
      if (initObj.hasOwnProperty('Path')) {
        this.Path = initObj.Path
      }
      else {
        this.Path = new nav_msgs.msg.Path();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoalToPathResponse
    // Serialize message field [Path]
    bufferOffset = nav_msgs.msg.Path.serialize(obj.Path, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoalToPathResponse
    let len;
    let data = new GoalToPathResponse(null);
    // Deserialize message field [Path]
    data.Path = nav_msgs.msg.Path.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nav_msgs.msg.Path.getMessageSize(object.Path);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dmp/GoalToPathResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24fae27c1f7b7e0732c341610d3780f7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    nav_msgs/Path Path
    
    
    ================================================================================
    MSG: nav_msgs/Path
    #An array of poses that represents a Path for a robot to follow
    Header header
    geometry_msgs/PoseStamped[] poses
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GoalToPathResponse(null);
    if (msg.Path !== undefined) {
      resolved.Path = nav_msgs.msg.Path.Resolve(msg.Path)
    }
    else {
      resolved.Path = new nav_msgs.msg.Path()
    }

    return resolved;
    }
};

module.exports = {
  Request: GoalToPathRequest,
  Response: GoalToPathResponse,
  md5sum() { return '61b7759cacf0fb0b224df3e235dda576'; },
  datatype() { return 'dmp/GoalToPath'; }
};
