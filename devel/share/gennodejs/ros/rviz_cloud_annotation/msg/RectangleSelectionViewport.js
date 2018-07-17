// Auto-generated. Do not edit!

// (in-package rviz_cloud_annotation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class RectangleSelectionViewport {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_x = null;
      this.start_y = null;
      this.end_x = null;
      this.end_y = null;
      this.viewport_height = null;
      this.viewport_width = null;
      this.focal_length = null;
      this.projection_matrix = null;
      this.camera_pose = null;
      this.is_deep_selection = null;
      this.polyline_x = null;
      this.polyline_y = null;
    }
    else {
      if (initObj.hasOwnProperty('start_x')) {
        this.start_x = initObj.start_x
      }
      else {
        this.start_x = 0;
      }
      if (initObj.hasOwnProperty('start_y')) {
        this.start_y = initObj.start_y
      }
      else {
        this.start_y = 0;
      }
      if (initObj.hasOwnProperty('end_x')) {
        this.end_x = initObj.end_x
      }
      else {
        this.end_x = 0;
      }
      if (initObj.hasOwnProperty('end_y')) {
        this.end_y = initObj.end_y
      }
      else {
        this.end_y = 0;
      }
      if (initObj.hasOwnProperty('viewport_height')) {
        this.viewport_height = initObj.viewport_height
      }
      else {
        this.viewport_height = 0;
      }
      if (initObj.hasOwnProperty('viewport_width')) {
        this.viewport_width = initObj.viewport_width
      }
      else {
        this.viewport_width = 0;
      }
      if (initObj.hasOwnProperty('focal_length')) {
        this.focal_length = initObj.focal_length
      }
      else {
        this.focal_length = 0.0;
      }
      if (initObj.hasOwnProperty('projection_matrix')) {
        this.projection_matrix = initObj.projection_matrix
      }
      else {
        this.projection_matrix = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('camera_pose')) {
        this.camera_pose = initObj.camera_pose
      }
      else {
        this.camera_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('is_deep_selection')) {
        this.is_deep_selection = initObj.is_deep_selection
      }
      else {
        this.is_deep_selection = false;
      }
      if (initObj.hasOwnProperty('polyline_x')) {
        this.polyline_x = initObj.polyline_x
      }
      else {
        this.polyline_x = [];
      }
      if (initObj.hasOwnProperty('polyline_y')) {
        this.polyline_y = initObj.polyline_y
      }
      else {
        this.polyline_y = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RectangleSelectionViewport
    // Serialize message field [start_x]
    bufferOffset = _serializer.uint32(obj.start_x, buffer, bufferOffset);
    // Serialize message field [start_y]
    bufferOffset = _serializer.uint32(obj.start_y, buffer, bufferOffset);
    // Serialize message field [end_x]
    bufferOffset = _serializer.uint32(obj.end_x, buffer, bufferOffset);
    // Serialize message field [end_y]
    bufferOffset = _serializer.uint32(obj.end_y, buffer, bufferOffset);
    // Serialize message field [viewport_height]
    bufferOffset = _serializer.uint32(obj.viewport_height, buffer, bufferOffset);
    // Serialize message field [viewport_width]
    bufferOffset = _serializer.uint32(obj.viewport_width, buffer, bufferOffset);
    // Serialize message field [focal_length]
    bufferOffset = _serializer.float32(obj.focal_length, buffer, bufferOffset);
    // Check that the constant length array field [projection_matrix] has the right length
    if (obj.projection_matrix.length !== 16) {
      throw new Error('Unable to serialize array field projection_matrix - length must be 16')
    }
    // Serialize message field [projection_matrix]
    bufferOffset = _arraySerializer.float32(obj.projection_matrix, buffer, bufferOffset, 16);
    // Serialize message field [camera_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.camera_pose, buffer, bufferOffset);
    // Serialize message field [is_deep_selection]
    bufferOffset = _serializer.bool(obj.is_deep_selection, buffer, bufferOffset);
    // Serialize message field [polyline_x]
    bufferOffset = _arraySerializer.int32(obj.polyline_x, buffer, bufferOffset, null);
    // Serialize message field [polyline_y]
    bufferOffset = _arraySerializer.int32(obj.polyline_y, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RectangleSelectionViewport
    let len;
    let data = new RectangleSelectionViewport(null);
    // Deserialize message field [start_x]
    data.start_x = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [start_y]
    data.start_y = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [end_x]
    data.end_x = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [end_y]
    data.end_y = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [viewport_height]
    data.viewport_height = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [viewport_width]
    data.viewport_width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [focal_length]
    data.focal_length = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [projection_matrix]
    data.projection_matrix = _arrayDeserializer.float32(buffer, bufferOffset, 16)
    // Deserialize message field [camera_pose]
    data.camera_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_deep_selection]
    data.is_deep_selection = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [polyline_x]
    data.polyline_x = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [polyline_y]
    data.polyline_y = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.polyline_x.length;
    length += 4 * object.polyline_y.length;
    return length + 157;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rviz_cloud_annotation/RectangleSelectionViewport';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a3c9a6075ac79ec934409411b5e99c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 start_x
    uint32 start_y
    uint32 end_x
    uint32 end_y
    
    uint32 viewport_height
    uint32 viewport_width
    
    float32 focal_length
    
    float32[16] projection_matrix
    geometry_msgs/Pose camera_pose
    
    bool is_deep_selection
    
    int32[] polyline_x
    int32[] polyline_y
    
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
    const resolved = new RectangleSelectionViewport(null);
    if (msg.start_x !== undefined) {
      resolved.start_x = msg.start_x;
    }
    else {
      resolved.start_x = 0
    }

    if (msg.start_y !== undefined) {
      resolved.start_y = msg.start_y;
    }
    else {
      resolved.start_y = 0
    }

    if (msg.end_x !== undefined) {
      resolved.end_x = msg.end_x;
    }
    else {
      resolved.end_x = 0
    }

    if (msg.end_y !== undefined) {
      resolved.end_y = msg.end_y;
    }
    else {
      resolved.end_y = 0
    }

    if (msg.viewport_height !== undefined) {
      resolved.viewport_height = msg.viewport_height;
    }
    else {
      resolved.viewport_height = 0
    }

    if (msg.viewport_width !== undefined) {
      resolved.viewport_width = msg.viewport_width;
    }
    else {
      resolved.viewport_width = 0
    }

    if (msg.focal_length !== undefined) {
      resolved.focal_length = msg.focal_length;
    }
    else {
      resolved.focal_length = 0.0
    }

    if (msg.projection_matrix !== undefined) {
      resolved.projection_matrix = msg.projection_matrix;
    }
    else {
      resolved.projection_matrix = new Array(16).fill(0)
    }

    if (msg.camera_pose !== undefined) {
      resolved.camera_pose = geometry_msgs.msg.Pose.Resolve(msg.camera_pose)
    }
    else {
      resolved.camera_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.is_deep_selection !== undefined) {
      resolved.is_deep_selection = msg.is_deep_selection;
    }
    else {
      resolved.is_deep_selection = false
    }

    if (msg.polyline_x !== undefined) {
      resolved.polyline_x = msg.polyline_x;
    }
    else {
      resolved.polyline_x = []
    }

    if (msg.polyline_y !== undefined) {
      resolved.polyline_y = msg.polyline_y;
    }
    else {
      resolved.polyline_y = []
    }

    return resolved;
    }
};

module.exports = RectangleSelectionViewport;
