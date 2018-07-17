
(cl:in-package :asdf)

(defsystem "rviz_cloud_annotation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "RectangleSelectionViewport" :depends-on ("_package_RectangleSelectionViewport"))
    (:file "_package_RectangleSelectionViewport" :depends-on ("_package"))
    (:file "UndoRedoState" :depends-on ("_package_UndoRedoState"))
    (:file "_package_UndoRedoState" :depends-on ("_package"))
  ))