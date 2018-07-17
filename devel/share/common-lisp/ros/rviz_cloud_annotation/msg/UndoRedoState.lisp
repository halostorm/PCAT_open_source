; Auto-generated. Do not edit!


(cl:in-package rviz_cloud_annotation-msg)


;//! \htmlinclude UndoRedoState.msg.html

(cl:defclass <UndoRedoState> (roslisp-msg-protocol:ros-message)
  ((redo_enabled
    :reader redo_enabled
    :initarg :redo_enabled
    :type cl:boolean
    :initform cl:nil)
   (redo_description
    :reader redo_description
    :initarg :redo_description
    :type cl:string
    :initform "")
   (undo_enabled
    :reader undo_enabled
    :initarg :undo_enabled
    :type cl:boolean
    :initform cl:nil)
   (undo_description
    :reader undo_description
    :initarg :undo_description
    :type cl:string
    :initform ""))
)

(cl:defclass UndoRedoState (<UndoRedoState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UndoRedoState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UndoRedoState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rviz_cloud_annotation-msg:<UndoRedoState> is deprecated: use rviz_cloud_annotation-msg:UndoRedoState instead.")))

(cl:ensure-generic-function 'redo_enabled-val :lambda-list '(m))
(cl:defmethod redo_enabled-val ((m <UndoRedoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:redo_enabled-val is deprecated.  Use rviz_cloud_annotation-msg:redo_enabled instead.")
  (redo_enabled m))

(cl:ensure-generic-function 'redo_description-val :lambda-list '(m))
(cl:defmethod redo_description-val ((m <UndoRedoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:redo_description-val is deprecated.  Use rviz_cloud_annotation-msg:redo_description instead.")
  (redo_description m))

(cl:ensure-generic-function 'undo_enabled-val :lambda-list '(m))
(cl:defmethod undo_enabled-val ((m <UndoRedoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:undo_enabled-val is deprecated.  Use rviz_cloud_annotation-msg:undo_enabled instead.")
  (undo_enabled m))

(cl:ensure-generic-function 'undo_description-val :lambda-list '(m))
(cl:defmethod undo_description-val ((m <UndoRedoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:undo_description-val is deprecated.  Use rviz_cloud_annotation-msg:undo_description instead.")
  (undo_description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UndoRedoState>) ostream)
  "Serializes a message object of type '<UndoRedoState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'redo_enabled) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'redo_description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'redo_description))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'undo_enabled) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'undo_description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'undo_description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UndoRedoState>) istream)
  "Deserializes a message object of type '<UndoRedoState>"
    (cl:setf (cl:slot-value msg 'redo_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'redo_description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'redo_description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'undo_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'undo_description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'undo_description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UndoRedoState>)))
  "Returns string type for a message object of type '<UndoRedoState>"
  "rviz_cloud_annotation/UndoRedoState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UndoRedoState)))
  "Returns string type for a message object of type 'UndoRedoState"
  "rviz_cloud_annotation/UndoRedoState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UndoRedoState>)))
  "Returns md5sum for a message object of type '<UndoRedoState>"
  "43c106a96c078080d8c117fdd425c0a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UndoRedoState)))
  "Returns md5sum for a message object of type 'UndoRedoState"
  "43c106a96c078080d8c117fdd425c0a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UndoRedoState>)))
  "Returns full string definition for message of type '<UndoRedoState>"
  (cl:format cl:nil "bool redo_enabled~%string redo_description~%~%bool undo_enabled~%string undo_description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UndoRedoState)))
  "Returns full string definition for message of type 'UndoRedoState"
  (cl:format cl:nil "bool redo_enabled~%string redo_description~%~%bool undo_enabled~%string undo_description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UndoRedoState>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'redo_description))
     1
     4 (cl:length (cl:slot-value msg 'undo_description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UndoRedoState>))
  "Converts a ROS message object to a list"
  (cl:list 'UndoRedoState
    (cl:cons ':redo_enabled (redo_enabled msg))
    (cl:cons ':redo_description (redo_description msg))
    (cl:cons ':undo_enabled (undo_enabled msg))
    (cl:cons ':undo_description (undo_description msg))
))
