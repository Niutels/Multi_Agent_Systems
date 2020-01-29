;; Auto-generated. Do not edit!


(when (boundp 'robot_sim::planner)
  (if (not (find-package "ROBOT_SIM"))
    (make-package "ROBOT_SIM"))
  (shadow 'planner (find-package "ROBOT_SIM")))
(unless (find-package "ROBOT_SIM::PLANNER")
  (make-package "ROBOT_SIM::PLANNER"))
(unless (find-package "ROBOT_SIM::PLANNERREQUEST")
  (make-package "ROBOT_SIM::PLANNERREQUEST"))
(unless (find-package "ROBOT_SIM::PLANNERRESPONSE")
  (make-package "ROBOT_SIM::PLANNERRESPONSE"))

(in-package "ROS")





(defclass robot_sim::plannerRequest
  :super ros::object
  :slots (_coords_msg ))

(defmethod robot_sim::plannerRequest
  (:init
   (&key
    ((:coords_msg __coords_msg) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _coords_msg __coords_msg)
   self)
  (:coords_msg
   (&optional __coords_msg)
   (if __coords_msg (setq _coords_msg __coords_msg)) _coords_msg)
  (:serialization-length
   ()
   (+
    ;; float32[] _coords_msg
    (* 4    (length _coords_msg)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _coords_msg
     (write-long (length _coords_msg) s)
     (dotimes (i (length _coords_msg))
       (sys::poke (elt _coords_msg i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _coords_msg
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coords_msg (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coords_msg i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass robot_sim::plannerResponse
  :super ros::object
  :slots (_optimal_path ))

(defmethod robot_sim::plannerResponse
  (:init
   (&key
    ((:optimal_path __optimal_path) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _optimal_path __optimal_path)
   self)
  (:optimal_path
   (&optional __optimal_path)
   (if __optimal_path (setq _optimal_path __optimal_path)) _optimal_path)
  (:serialization-length
   ()
   (+
    ;; float32[] _optimal_path
    (* 4    (length _optimal_path)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _optimal_path
     (write-long (length _optimal_path) s)
     (dotimes (i (length _optimal_path))
       (sys::poke (elt _optimal_path i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _optimal_path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _optimal_path (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _optimal_path i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass robot_sim::planner
  :super ros::object
  :slots ())

(setf (get robot_sim::planner :md5sum-) "6423f791599f0a66e62771c89cda1dcd")
(setf (get robot_sim::planner :datatype-) "robot_sim/planner")
(setf (get robot_sim::planner :request) robot_sim::plannerRequest)
(setf (get robot_sim::planner :response) robot_sim::plannerResponse)

(defmethod robot_sim::plannerRequest
  (:response () (instance robot_sim::plannerResponse :init)))

(setf (get robot_sim::plannerRequest :md5sum-) "6423f791599f0a66e62771c89cda1dcd")
(setf (get robot_sim::plannerRequest :datatype-) "robot_sim/plannerRequest")
(setf (get robot_sim::plannerRequest :definition-)
      "float32[] coords_msg
---
float32[] optimal_path
")

(setf (get robot_sim::plannerResponse :md5sum-) "6423f791599f0a66e62771c89cda1dcd")
(setf (get robot_sim::plannerResponse :datatype-) "robot_sim/plannerResponse")
(setf (get robot_sim::plannerResponse :definition-)
      "float32[] coords_msg
---
float32[] optimal_path
")



(provide :robot_sim/planner "6423f791599f0a66e62771c89cda1dcd")


