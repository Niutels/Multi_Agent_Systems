
(cl:in-package :asdf)

(defsystem "robot_sim-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "other_task" :depends-on ("_package_other_task"))
    (:file "_package_other_task" :depends-on ("_package"))
    (:file "pos_task" :depends-on ("_package_pos_task"))
    (:file "_package_pos_task" :depends-on ("_package"))
  ))