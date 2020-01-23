;; Auto-generated. Do not edit!


(when (boundp 'robot_sim::pos_task)
  (if (not (find-package "ROBOT_SIM"))
    (make-package "ROBOT_SIM"))
  (shadow 'pos_task (find-package "ROBOT_SIM")))
(unless (find-package "ROBOT_SIM::POS_TASK")
  (make-package "ROBOT_SIM::POS_TASK"))
(unless (find-package "ROBOT_SIM::POS_TASKREQUEST")
  (make-package "ROBOT_SIM::POS_TASKREQUEST"))
(unless (find-package "ROBOT_SIM::POS_TASKRESPONSE")
  (make-package "ROBOT_SIM::POS_TASKRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass robot_sim::pos_taskRequest
  :super ros::object
  :slots (_task_data _task_type _task_id ))

(defmethod robot_sim::pos_taskRequest
  (:init
   (&key
    ((:task_data __task_data) (instance geometry_msgs::Pose2D :init))
    ((:task_type __task_type) "")
    ((:task_id __task_id) 0)
    )
   (send-super :init)
   (setq _task_data __task_data)
   (setq _task_type (string __task_type))
   (setq _task_id (round __task_id))
   self)
  (:task_data
   (&rest __task_data)
   (if (keywordp (car __task_data))
       (send* _task_data __task_data)
     (progn
       (if __task_data (setq _task_data (car __task_data)))
       _task_data)))
  (:task_type
   (&optional __task_type)
   (if __task_type (setq _task_type __task_type)) _task_type)
  (:task_id
   (&optional __task_id)
   (if __task_id (setq _task_id __task_id)) _task_id)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose2D _task_data
    (send _task_data :serialization-length)
    ;; string _task_type
    4 (length _task_type)
    ;; int64 _task_id
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose2D _task_data
       (send _task_data :serialize s)
     ;; string _task_type
       (write-long (length _task_type) s) (princ _task_type s)
     ;; int64 _task_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _task_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _task_id) (= (length (_task_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_task_id . bv) 0) 0) s)
              (write-long (ash (elt (_task_id . bv) 1) -1) s))
             ((and (class _task_id) (= (length (_task_id . bv)) 1)) ;; big1
              (write-long (elt (_task_id . bv) 0) s)
              (write-long (if (>= _task_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _task_id s)(write-long (if (>= _task_id 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose2D _task_data
     (send _task_data :deserialize buf ptr-) (incf ptr- (send _task_data :serialization-length))
   ;; string _task_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _task_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _task_id
#+(or :alpha :irix6 :x86_64)
      (setf _task_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _task_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass robot_sim::pos_taskResponse
  :super ros::object
  :slots ())

(defmethod robot_sim::pos_taskResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass robot_sim::pos_task
  :super ros::object
  :slots ())

(setf (get robot_sim::pos_task :md5sum-) "1998cd4f5d6808303435266496f15d7e")
(setf (get robot_sim::pos_task :datatype-) "robot_sim/pos_task")
(setf (get robot_sim::pos_task :request) robot_sim::pos_taskRequest)
(setf (get robot_sim::pos_task :response) robot_sim::pos_taskResponse)

(defmethod robot_sim::pos_taskRequest
  (:response () (instance robot_sim::pos_taskResponse :init)))

(setf (get robot_sim::pos_taskRequest :md5sum-) "1998cd4f5d6808303435266496f15d7e")
(setf (get robot_sim::pos_taskRequest :datatype-) "robot_sim/pos_taskRequest")
(setf (get robot_sim::pos_taskRequest :definition-)
      "geometry_msgs/Pose2D task_data
string task_type
int64 task_id


================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
---
")

(setf (get robot_sim::pos_taskResponse :md5sum-) "1998cd4f5d6808303435266496f15d7e")
(setf (get robot_sim::pos_taskResponse :datatype-) "robot_sim/pos_taskResponse")
(setf (get robot_sim::pos_taskResponse :definition-)
      "geometry_msgs/Pose2D task_data
string task_type
int64 task_id


================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
---
")



(provide :robot_sim/pos_task "1998cd4f5d6808303435266496f15d7e")


