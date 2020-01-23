;; Auto-generated. Do not edit!


(when (boundp 'robot_sim::other_task)
  (if (not (find-package "ROBOT_SIM"))
    (make-package "ROBOT_SIM"))
  (shadow 'other_task (find-package "ROBOT_SIM")))
(unless (find-package "ROBOT_SIM::OTHER_TASK")
  (make-package "ROBOT_SIM::OTHER_TASK"))
(unless (find-package "ROBOT_SIM::OTHER_TASKREQUEST")
  (make-package "ROBOT_SIM::OTHER_TASKREQUEST"))
(unless (find-package "ROBOT_SIM::OTHER_TASKRESPONSE")
  (make-package "ROBOT_SIM::OTHER_TASKRESPONSE"))

(in-package "ROS")





(defclass robot_sim::other_taskRequest
  :super ros::object
  :slots (_task_data _task_type _task_id ))

(defmethod robot_sim::other_taskRequest
  (:init
   (&key
    ((:task_data __task_data) 0)
    ((:task_type __task_type) "")
    ((:task_id __task_id) 0)
    )
   (send-super :init)
   (setq _task_data (round __task_data))
   (setq _task_type (string __task_type))
   (setq _task_id (round __task_id))
   self)
  (:task_data
   (&optional __task_data)
   (if __task_data (setq _task_data __task_data)) _task_data)
  (:task_type
   (&optional __task_type)
   (if __task_type (setq _task_type __task_type)) _task_type)
  (:task_id
   (&optional __task_id)
   (if __task_id (setq _task_id __task_id)) _task_id)
  (:serialization-length
   ()
   (+
    ;; int32 _task_data
    4
    ;; string _task_type
    4 (length _task_type)
    ;; int64 _task_id
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _task_data
       (write-long _task_data s)
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
   ;; int32 _task_data
     (setq _task_data (sys::peek buf ptr- :integer)) (incf ptr- 4)
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

(defclass robot_sim::other_taskResponse
  :super ros::object
  :slots ())

(defmethod robot_sim::other_taskResponse
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

(defclass robot_sim::other_task
  :super ros::object
  :slots ())

(setf (get robot_sim::other_task :md5sum-) "d0f539cb3e5e23bc53002dc7b1f24555")
(setf (get robot_sim::other_task :datatype-) "robot_sim/other_task")
(setf (get robot_sim::other_task :request) robot_sim::other_taskRequest)
(setf (get robot_sim::other_task :response) robot_sim::other_taskResponse)

(defmethod robot_sim::other_taskRequest
  (:response () (instance robot_sim::other_taskResponse :init)))

(setf (get robot_sim::other_taskRequest :md5sum-) "d0f539cb3e5e23bc53002dc7b1f24555")
(setf (get robot_sim::other_taskRequest :datatype-) "robot_sim/other_taskRequest")
(setf (get robot_sim::other_taskRequest :definition-)
      "int32 task_data
string task_type
int64 task_id

---
")

(setf (get robot_sim::other_taskResponse :md5sum-) "d0f539cb3e5e23bc53002dc7b1f24555")
(setf (get robot_sim::other_taskResponse :datatype-) "robot_sim/other_taskResponse")
(setf (get robot_sim::other_taskResponse :definition-)
      "int32 task_data
string task_type
int64 task_id

---
")



(provide :robot_sim/other_task "d0f539cb3e5e23bc53002dc7b1f24555")


