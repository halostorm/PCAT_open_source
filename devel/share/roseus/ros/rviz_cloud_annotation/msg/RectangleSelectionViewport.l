;; Auto-generated. Do not edit!


(when (boundp 'rviz_cloud_annotation::RectangleSelectionViewport)
  (if (not (find-package "RVIZ_CLOUD_ANNOTATION"))
    (make-package "RVIZ_CLOUD_ANNOTATION"))
  (shadow 'RectangleSelectionViewport (find-package "RVIZ_CLOUD_ANNOTATION")))
(unless (find-package "RVIZ_CLOUD_ANNOTATION::RECTANGLESELECTIONVIEWPORT")
  (make-package "RVIZ_CLOUD_ANNOTATION::RECTANGLESELECTIONVIEWPORT"))

(in-package "ROS")
;;//! \htmlinclude RectangleSelectionViewport.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass rviz_cloud_annotation::RectangleSelectionViewport
  :super ros::object
  :slots (_start_x _start_y _end_x _end_y _viewport_height _viewport_width _focal_length _projection_matrix _camera_pose _is_deep_selection _polyline_x _polyline_y ))

(defmethod rviz_cloud_annotation::RectangleSelectionViewport
  (:init
   (&key
    ((:start_x __start_x) 0)
    ((:start_y __start_y) 0)
    ((:end_x __end_x) 0)
    ((:end_y __end_y) 0)
    ((:viewport_height __viewport_height) 0)
    ((:viewport_width __viewport_width) 0)
    ((:focal_length __focal_length) 0.0)
    ((:projection_matrix __projection_matrix) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:camera_pose __camera_pose) (instance geometry_msgs::Pose :init))
    ((:is_deep_selection __is_deep_selection) nil)
    ((:polyline_x __polyline_x) (make-array 0 :initial-element 0 :element-type :integer))
    ((:polyline_y __polyline_y) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _start_x (round __start_x))
   (setq _start_y (round __start_y))
   (setq _end_x (round __end_x))
   (setq _end_y (round __end_y))
   (setq _viewport_height (round __viewport_height))
   (setq _viewport_width (round __viewport_width))
   (setq _focal_length (float __focal_length))
   (setq _projection_matrix __projection_matrix)
   (setq _camera_pose __camera_pose)
   (setq _is_deep_selection __is_deep_selection)
   (setq _polyline_x __polyline_x)
   (setq _polyline_y __polyline_y)
   self)
  (:start_x
   (&optional __start_x)
   (if __start_x (setq _start_x __start_x)) _start_x)
  (:start_y
   (&optional __start_y)
   (if __start_y (setq _start_y __start_y)) _start_y)
  (:end_x
   (&optional __end_x)
   (if __end_x (setq _end_x __end_x)) _end_x)
  (:end_y
   (&optional __end_y)
   (if __end_y (setq _end_y __end_y)) _end_y)
  (:viewport_height
   (&optional __viewport_height)
   (if __viewport_height (setq _viewport_height __viewport_height)) _viewport_height)
  (:viewport_width
   (&optional __viewport_width)
   (if __viewport_width (setq _viewport_width __viewport_width)) _viewport_width)
  (:focal_length
   (&optional __focal_length)
   (if __focal_length (setq _focal_length __focal_length)) _focal_length)
  (:projection_matrix
   (&optional __projection_matrix)
   (if __projection_matrix (setq _projection_matrix __projection_matrix)) _projection_matrix)
  (:camera_pose
   (&rest __camera_pose)
   (if (keywordp (car __camera_pose))
       (send* _camera_pose __camera_pose)
     (progn
       (if __camera_pose (setq _camera_pose (car __camera_pose)))
       _camera_pose)))
  (:is_deep_selection
   (&optional __is_deep_selection)
   (if __is_deep_selection (setq _is_deep_selection __is_deep_selection)) _is_deep_selection)
  (:polyline_x
   (&optional __polyline_x)
   (if __polyline_x (setq _polyline_x __polyline_x)) _polyline_x)
  (:polyline_y
   (&optional __polyline_y)
   (if __polyline_y (setq _polyline_y __polyline_y)) _polyline_y)
  (:serialization-length
   ()
   (+
    ;; uint32 _start_x
    4
    ;; uint32 _start_y
    4
    ;; uint32 _end_x
    4
    ;; uint32 _end_y
    4
    ;; uint32 _viewport_height
    4
    ;; uint32 _viewport_width
    4
    ;; float32 _focal_length
    4
    ;; float32[16] _projection_matrix
    (* 4    16)
    ;; geometry_msgs/Pose _camera_pose
    (send _camera_pose :serialization-length)
    ;; bool _is_deep_selection
    1
    ;; int32[] _polyline_x
    (* 4    (length _polyline_x)) 4
    ;; int32[] _polyline_y
    (* 4    (length _polyline_y)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _start_x
       (write-long _start_x s)
     ;; uint32 _start_y
       (write-long _start_y s)
     ;; uint32 _end_x
       (write-long _end_x s)
     ;; uint32 _end_y
       (write-long _end_y s)
     ;; uint32 _viewport_height
       (write-long _viewport_height s)
     ;; uint32 _viewport_width
       (write-long _viewport_width s)
     ;; float32 _focal_length
       (sys::poke _focal_length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[16] _projection_matrix
     (dotimes (i 16)
       (sys::poke (elt _projection_matrix i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; geometry_msgs/Pose _camera_pose
       (send _camera_pose :serialize s)
     ;; bool _is_deep_selection
       (if _is_deep_selection (write-byte -1 s) (write-byte 0 s))
     ;; int32[] _polyline_x
     (write-long (length _polyline_x) s)
     (dotimes (i (length _polyline_x))
       (write-long (elt _polyline_x i) s)
       )
     ;; int32[] _polyline_y
     (write-long (length _polyline_y) s)
     (dotimes (i (length _polyline_y))
       (write-long (elt _polyline_y i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _start_x
     (setq _start_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _start_y
     (setq _start_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _end_x
     (setq _end_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _end_y
     (setq _end_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _viewport_height
     (setq _viewport_height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _viewport_width
     (setq _viewport_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _focal_length
     (setq _focal_length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[16] _projection_matrix
   (dotimes (i (length _projection_matrix))
     (setf (elt _projection_matrix i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; geometry_msgs/Pose _camera_pose
     (send _camera_pose :deserialize buf ptr-) (incf ptr- (send _camera_pose :serialization-length))
   ;; bool _is_deep_selection
     (setq _is_deep_selection (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32[] _polyline_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _polyline_x (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _polyline_x i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _polyline_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _polyline_y (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _polyline_y i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get rviz_cloud_annotation::RectangleSelectionViewport :md5sum-) "6a3c9a6075ac79ec934409411b5e99c9")
(setf (get rviz_cloud_annotation::RectangleSelectionViewport :datatype-) "rviz_cloud_annotation/RectangleSelectionViewport")
(setf (get rviz_cloud_annotation::RectangleSelectionViewport :definition-)
      "uint32 start_x
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

")



(provide :rviz_cloud_annotation/RectangleSelectionViewport "6a3c9a6075ac79ec934409411b5e99c9")


