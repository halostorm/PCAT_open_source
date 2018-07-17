; Auto-generated. Do not edit!


(cl:in-package rviz_cloud_annotation-msg)


;//! \htmlinclude RectangleSelectionViewport.msg.html

(cl:defclass <RectangleSelectionViewport> (roslisp-msg-protocol:ros-message)
  ((start_x
    :reader start_x
    :initarg :start_x
    :type cl:integer
    :initform 0)
   (start_y
    :reader start_y
    :initarg :start_y
    :type cl:integer
    :initform 0)
   (end_x
    :reader end_x
    :initarg :end_x
    :type cl:integer
    :initform 0)
   (end_y
    :reader end_y
    :initarg :end_y
    :type cl:integer
    :initform 0)
   (viewport_height
    :reader viewport_height
    :initarg :viewport_height
    :type cl:integer
    :initform 0)
   (viewport_width
    :reader viewport_width
    :initarg :viewport_width
    :type cl:integer
    :initform 0)
   (focal_length
    :reader focal_length
    :initarg :focal_length
    :type cl:float
    :initform 0.0)
   (projection_matrix
    :reader projection_matrix
    :initarg :projection_matrix
    :type (cl:vector cl:float)
   :initform (cl:make-array 16 :element-type 'cl:float :initial-element 0.0))
   (camera_pose
    :reader camera_pose
    :initarg :camera_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (is_deep_selection
    :reader is_deep_selection
    :initarg :is_deep_selection
    :type cl:boolean
    :initform cl:nil)
   (polyline_x
    :reader polyline_x
    :initarg :polyline_x
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (polyline_y
    :reader polyline_y
    :initarg :polyline_y
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass RectangleSelectionViewport (<RectangleSelectionViewport>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RectangleSelectionViewport>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RectangleSelectionViewport)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rviz_cloud_annotation-msg:<RectangleSelectionViewport> is deprecated: use rviz_cloud_annotation-msg:RectangleSelectionViewport instead.")))

(cl:ensure-generic-function 'start_x-val :lambda-list '(m))
(cl:defmethod start_x-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:start_x-val is deprecated.  Use rviz_cloud_annotation-msg:start_x instead.")
  (start_x m))

(cl:ensure-generic-function 'start_y-val :lambda-list '(m))
(cl:defmethod start_y-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:start_y-val is deprecated.  Use rviz_cloud_annotation-msg:start_y instead.")
  (start_y m))

(cl:ensure-generic-function 'end_x-val :lambda-list '(m))
(cl:defmethod end_x-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:end_x-val is deprecated.  Use rviz_cloud_annotation-msg:end_x instead.")
  (end_x m))

(cl:ensure-generic-function 'end_y-val :lambda-list '(m))
(cl:defmethod end_y-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:end_y-val is deprecated.  Use rviz_cloud_annotation-msg:end_y instead.")
  (end_y m))

(cl:ensure-generic-function 'viewport_height-val :lambda-list '(m))
(cl:defmethod viewport_height-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:viewport_height-val is deprecated.  Use rviz_cloud_annotation-msg:viewport_height instead.")
  (viewport_height m))

(cl:ensure-generic-function 'viewport_width-val :lambda-list '(m))
(cl:defmethod viewport_width-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:viewport_width-val is deprecated.  Use rviz_cloud_annotation-msg:viewport_width instead.")
  (viewport_width m))

(cl:ensure-generic-function 'focal_length-val :lambda-list '(m))
(cl:defmethod focal_length-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:focal_length-val is deprecated.  Use rviz_cloud_annotation-msg:focal_length instead.")
  (focal_length m))

(cl:ensure-generic-function 'projection_matrix-val :lambda-list '(m))
(cl:defmethod projection_matrix-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:projection_matrix-val is deprecated.  Use rviz_cloud_annotation-msg:projection_matrix instead.")
  (projection_matrix m))

(cl:ensure-generic-function 'camera_pose-val :lambda-list '(m))
(cl:defmethod camera_pose-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:camera_pose-val is deprecated.  Use rviz_cloud_annotation-msg:camera_pose instead.")
  (camera_pose m))

(cl:ensure-generic-function 'is_deep_selection-val :lambda-list '(m))
(cl:defmethod is_deep_selection-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:is_deep_selection-val is deprecated.  Use rviz_cloud_annotation-msg:is_deep_selection instead.")
  (is_deep_selection m))

(cl:ensure-generic-function 'polyline_x-val :lambda-list '(m))
(cl:defmethod polyline_x-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:polyline_x-val is deprecated.  Use rviz_cloud_annotation-msg:polyline_x instead.")
  (polyline_x m))

(cl:ensure-generic-function 'polyline_y-val :lambda-list '(m))
(cl:defmethod polyline_y-val ((m <RectangleSelectionViewport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz_cloud_annotation-msg:polyline_y-val is deprecated.  Use rviz_cloud_annotation-msg:polyline_y instead.")
  (polyline_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RectangleSelectionViewport>) ostream)
  "Serializes a message object of type '<RectangleSelectionViewport>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'start_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'start_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'start_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'start_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'start_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'start_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'start_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'start_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'end_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'end_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'end_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'end_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'end_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'end_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'viewport_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'viewport_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'viewport_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'viewport_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'viewport_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'viewport_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'viewport_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'viewport_width)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'focal_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'projection_matrix))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'camera_pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_deep_selection) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'polyline_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'polyline_x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'polyline_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'polyline_y))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RectangleSelectionViewport>) istream)
  "Deserializes a message object of type '<RectangleSelectionViewport>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'start_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'start_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'start_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'start_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'start_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'start_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'start_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'start_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'end_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'end_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'end_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'end_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'end_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'end_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'viewport_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'viewport_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'viewport_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'viewport_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'viewport_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'viewport_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'viewport_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'viewport_width)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'focal_length) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'projection_matrix) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'projection_matrix)))
    (cl:dotimes (i 16)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'camera_pose) istream)
    (cl:setf (cl:slot-value msg 'is_deep_selection) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'polyline_x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'polyline_x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'polyline_y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'polyline_y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RectangleSelectionViewport>)))
  "Returns string type for a message object of type '<RectangleSelectionViewport>"
  "rviz_cloud_annotation/RectangleSelectionViewport")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RectangleSelectionViewport)))
  "Returns string type for a message object of type 'RectangleSelectionViewport"
  "rviz_cloud_annotation/RectangleSelectionViewport")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RectangleSelectionViewport>)))
  "Returns md5sum for a message object of type '<RectangleSelectionViewport>"
  "6a3c9a6075ac79ec934409411b5e99c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RectangleSelectionViewport)))
  "Returns md5sum for a message object of type 'RectangleSelectionViewport"
  "6a3c9a6075ac79ec934409411b5e99c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RectangleSelectionViewport>)))
  "Returns full string definition for message of type '<RectangleSelectionViewport>"
  (cl:format cl:nil "uint32 start_x~%uint32 start_y~%uint32 end_x~%uint32 end_y~%~%uint32 viewport_height~%uint32 viewport_width~%~%float32 focal_length~%~%float32[16] projection_matrix~%geometry_msgs/Pose camera_pose~%~%bool is_deep_selection~%~%int32[] polyline_x~%int32[] polyline_y~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RectangleSelectionViewport)))
  "Returns full string definition for message of type 'RectangleSelectionViewport"
  (cl:format cl:nil "uint32 start_x~%uint32 start_y~%uint32 end_x~%uint32 end_y~%~%uint32 viewport_height~%uint32 viewport_width~%~%float32 focal_length~%~%float32[16] projection_matrix~%geometry_msgs/Pose camera_pose~%~%bool is_deep_selection~%~%int32[] polyline_x~%int32[] polyline_y~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RectangleSelectionViewport>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'projection_matrix) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'camera_pose))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'polyline_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'polyline_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RectangleSelectionViewport>))
  "Converts a ROS message object to a list"
  (cl:list 'RectangleSelectionViewport
    (cl:cons ':start_x (start_x msg))
    (cl:cons ':start_y (start_y msg))
    (cl:cons ':end_x (end_x msg))
    (cl:cons ':end_y (end_y msg))
    (cl:cons ':viewport_height (viewport_height msg))
    (cl:cons ':viewport_width (viewport_width msg))
    (cl:cons ':focal_length (focal_length msg))
    (cl:cons ':projection_matrix (projection_matrix msg))
    (cl:cons ':camera_pose (camera_pose msg))
    (cl:cons ':is_deep_selection (is_deep_selection msg))
    (cl:cons ':polyline_x (polyline_x msg))
    (cl:cons ':polyline_y (polyline_y msg))
))
