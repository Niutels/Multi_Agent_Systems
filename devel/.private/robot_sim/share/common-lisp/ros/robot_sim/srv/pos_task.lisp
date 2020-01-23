; Auto-generated. Do not edit!


(cl:in-package robot_sim-srv)


;//! \htmlinclude pos_task-request.msg.html

(cl:defclass <pos_task-request> (roslisp-msg-protocol:ros-message)
  ((task_data
    :reader task_data
    :initarg :task_data
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (task_type
    :reader task_type
    :initarg :task_type
    :type cl:string
    :initform "")
   (task_id
    :reader task_id
    :initarg :task_id
    :type cl:integer
    :initform 0))
)

(cl:defclass pos_task-request (<pos_task-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos_task-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos_task-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_sim-srv:<pos_task-request> is deprecated: use robot_sim-srv:pos_task-request instead.")))

(cl:ensure-generic-function 'task_data-val :lambda-list '(m))
(cl:defmethod task_data-val ((m <pos_task-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_sim-srv:task_data-val is deprecated.  Use robot_sim-srv:task_data instead.")
  (task_data m))

(cl:ensure-generic-function 'task_type-val :lambda-list '(m))
(cl:defmethod task_type-val ((m <pos_task-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_sim-srv:task_type-val is deprecated.  Use robot_sim-srv:task_type instead.")
  (task_type m))

(cl:ensure-generic-function 'task_id-val :lambda-list '(m))
(cl:defmethod task_id-val ((m <pos_task-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_sim-srv:task_id-val is deprecated.  Use robot_sim-srv:task_id instead.")
  (task_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos_task-request>) ostream)
  "Serializes a message object of type '<pos_task-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'task_data) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'task_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'task_type))
  (cl:let* ((signed (cl:slot-value msg 'task_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos_task-request>) istream)
  "Deserializes a message object of type '<pos_task-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'task_data) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'task_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos_task-request>)))
  "Returns string type for a service object of type '<pos_task-request>"
  "robot_sim/pos_taskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos_task-request)))
  "Returns string type for a service object of type 'pos_task-request"
  "robot_sim/pos_taskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos_task-request>)))
  "Returns md5sum for a message object of type '<pos_task-request>"
  "1998cd4f5d6808303435266496f15d7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos_task-request)))
  "Returns md5sum for a message object of type 'pos_task-request"
  "1998cd4f5d6808303435266496f15d7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos_task-request>)))
  "Returns full string definition for message of type '<pos_task-request>"
  (cl:format cl:nil "geometry_msgs/Pose2D task_data~%string task_type~%int64 task_id~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos_task-request)))
  "Returns full string definition for message of type 'pos_task-request"
  (cl:format cl:nil "geometry_msgs/Pose2D task_data~%string task_type~%int64 task_id~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos_task-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'task_data))
     4 (cl:length (cl:slot-value msg 'task_type))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos_task-request>))
  "Converts a ROS message object to a list"
  (cl:list 'pos_task-request
    (cl:cons ':task_data (task_data msg))
    (cl:cons ':task_type (task_type msg))
    (cl:cons ':task_id (task_id msg))
))
;//! \htmlinclude pos_task-response.msg.html

(cl:defclass <pos_task-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass pos_task-response (<pos_task-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos_task-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos_task-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_sim-srv:<pos_task-response> is deprecated: use robot_sim-srv:pos_task-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos_task-response>) ostream)
  "Serializes a message object of type '<pos_task-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos_task-response>) istream)
  "Deserializes a message object of type '<pos_task-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos_task-response>)))
  "Returns string type for a service object of type '<pos_task-response>"
  "robot_sim/pos_taskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos_task-response)))
  "Returns string type for a service object of type 'pos_task-response"
  "robot_sim/pos_taskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos_task-response>)))
  "Returns md5sum for a message object of type '<pos_task-response>"
  "1998cd4f5d6808303435266496f15d7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos_task-response)))
  "Returns md5sum for a message object of type 'pos_task-response"
  "1998cd4f5d6808303435266496f15d7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos_task-response>)))
  "Returns full string definition for message of type '<pos_task-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos_task-response)))
  "Returns full string definition for message of type 'pos_task-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos_task-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos_task-response>))
  "Converts a ROS message object to a list"
  (cl:list 'pos_task-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'pos_task)))
  'pos_task-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'pos_task)))
  'pos_task-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos_task)))
  "Returns string type for a service object of type '<pos_task>"
  "robot_sim/pos_task")