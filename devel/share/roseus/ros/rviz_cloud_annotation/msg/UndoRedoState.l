;; Auto-generated. Do not edit!


(when (boundp 'rviz_cloud_annotation::UndoRedoState)
  (if (not (find-package "RVIZ_CLOUD_ANNOTATION"))
    (make-package "RVIZ_CLOUD_ANNOTATION"))
  (shadow 'UndoRedoState (find-package "RVIZ_CLOUD_ANNOTATION")))
(unless (find-package "RVIZ_CLOUD_ANNOTATION::UNDOREDOSTATE")
  (make-package "RVIZ_CLOUD_ANNOTATION::UNDOREDOSTATE"))

(in-package "ROS")
;;//! \htmlinclude UndoRedoState.msg.html


(defclass rviz_cloud_annotation::UndoRedoState
  :super ros::object
  :slots (_redo_enabled _redo_description _undo_enabled _undo_description ))

(defmethod rviz_cloud_annotation::UndoRedoState
  (:init
   (&key
    ((:redo_enabled __redo_enabled) nil)
    ((:redo_description __redo_description) "")
    ((:undo_enabled __undo_enabled) nil)
    ((:undo_description __undo_description) "")
    )
   (send-super :init)
   (setq _redo_enabled __redo_enabled)
   (setq _redo_description (string __redo_description))
   (setq _undo_enabled __undo_enabled)
   (setq _undo_description (string __undo_description))
   self)
  (:redo_enabled
   (&optional __redo_enabled)
   (if __redo_enabled (setq _redo_enabled __redo_enabled)) _redo_enabled)
  (:redo_description
   (&optional __redo_description)
   (if __redo_description (setq _redo_description __redo_description)) _redo_description)
  (:undo_enabled
   (&optional __undo_enabled)
   (if __undo_enabled (setq _undo_enabled __undo_enabled)) _undo_enabled)
  (:undo_description
   (&optional __undo_description)
   (if __undo_description (setq _undo_description __undo_description)) _undo_description)
  (:serialization-length
   ()
   (+
    ;; bool _redo_enabled
    1
    ;; string _redo_description
    4 (length _redo_description)
    ;; bool _undo_enabled
    1
    ;; string _undo_description
    4 (length _undo_description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _redo_enabled
       (if _redo_enabled (write-byte -1 s) (write-byte 0 s))
     ;; string _redo_description
       (write-long (length _redo_description) s) (princ _redo_description s)
     ;; bool _undo_enabled
       (if _undo_enabled (write-byte -1 s) (write-byte 0 s))
     ;; string _undo_description
       (write-long (length _undo_description) s) (princ _undo_description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _redo_enabled
     (setq _redo_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _redo_description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _redo_description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _undo_enabled
     (setq _undo_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _undo_description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _undo_description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get rviz_cloud_annotation::UndoRedoState :md5sum-) "43c106a96c078080d8c117fdd425c0a0")
(setf (get rviz_cloud_annotation::UndoRedoState :datatype-) "rviz_cloud_annotation/UndoRedoState")
(setf (get rviz_cloud_annotation::UndoRedoState :definition-)
      "bool redo_enabled
string redo_description

bool undo_enabled
string undo_description

")



(provide :rviz_cloud_annotation/UndoRedoState "43c106a96c078080d8c117fdd425c0a0")


