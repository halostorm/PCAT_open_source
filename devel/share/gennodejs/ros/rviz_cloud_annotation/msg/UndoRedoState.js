// Auto-generated. Do not edit!

// (in-package rviz_cloud_annotation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class UndoRedoState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.redo_enabled = null;
      this.redo_description = null;
      this.undo_enabled = null;
      this.undo_description = null;
    }
    else {
      if (initObj.hasOwnProperty('redo_enabled')) {
        this.redo_enabled = initObj.redo_enabled
      }
      else {
        this.redo_enabled = false;
      }
      if (initObj.hasOwnProperty('redo_description')) {
        this.redo_description = initObj.redo_description
      }
      else {
        this.redo_description = '';
      }
      if (initObj.hasOwnProperty('undo_enabled')) {
        this.undo_enabled = initObj.undo_enabled
      }
      else {
        this.undo_enabled = false;
      }
      if (initObj.hasOwnProperty('undo_description')) {
        this.undo_description = initObj.undo_description
      }
      else {
        this.undo_description = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UndoRedoState
    // Serialize message field [redo_enabled]
    bufferOffset = _serializer.bool(obj.redo_enabled, buffer, bufferOffset);
    // Serialize message field [redo_description]
    bufferOffset = _serializer.string(obj.redo_description, buffer, bufferOffset);
    // Serialize message field [undo_enabled]
    bufferOffset = _serializer.bool(obj.undo_enabled, buffer, bufferOffset);
    // Serialize message field [undo_description]
    bufferOffset = _serializer.string(obj.undo_description, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UndoRedoState
    let len;
    let data = new UndoRedoState(null);
    // Deserialize message field [redo_enabled]
    data.redo_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [redo_description]
    data.redo_description = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [undo_enabled]
    data.undo_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [undo_description]
    data.undo_description = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.redo_description.length;
    length += object.undo_description.length;
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rviz_cloud_annotation/UndoRedoState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '43c106a96c078080d8c117fdd425c0a0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool redo_enabled
    string redo_description
    
    bool undo_enabled
    string undo_description
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UndoRedoState(null);
    if (msg.redo_enabled !== undefined) {
      resolved.redo_enabled = msg.redo_enabled;
    }
    else {
      resolved.redo_enabled = false
    }

    if (msg.redo_description !== undefined) {
      resolved.redo_description = msg.redo_description;
    }
    else {
      resolved.redo_description = ''
    }

    if (msg.undo_enabled !== undefined) {
      resolved.undo_enabled = msg.undo_enabled;
    }
    else {
      resolved.undo_enabled = false
    }

    if (msg.undo_description !== undefined) {
      resolved.undo_description = msg.undo_description;
    }
    else {
      resolved.undo_description = ''
    }

    return resolved;
    }
};

module.exports = UndoRedoState;
