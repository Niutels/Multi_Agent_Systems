; Auto-generated. Do not edit!


(cl:in-package robot_sim-srv)


;//! \htmlinclude other_task-request.msg.html

(cl:defclass <other_task-request> (roslisp-msg-protocol:ros-message)
  ((task_data
    :reader task_data
    :initarg :task_data
    :type cl:integer
    :initform 0)
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

(cl:defclass other_task-request (<other_task-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <other_task-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'other_task-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_sim-srv:<other_task-request> is deprecated: use robot_sim-srv:other_task-request instead.")))

(cl:ensure-generic-function 'task_data-val :lambda-list '(m))
(cl:defmethod task_data-val ((m <other_task-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_sim-srv:task_data-val is deprecated.  Use robot_sim-srv:task_data instead.")
  (task_data m))

(cl:ensure-generic-function 'task_type-val :lambda-list '(m))
(cl:defmethod task_type-val ((m <other_task-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_sim-srv:task_type-val is deprecated.  Use robot_sim-srv:task_type instead.")
  (task_type m))

(cl:ensure-generic-function 'task_id-val :lambda-list '(m))
(cl:defmethod task_id-val ((m <other_task-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_sim-srv:task_id-val is deprecated.  Use robot_sim-srv:task_id instead.")
  (task_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <other_task-request>) ostream)
  "Serializes a message object of type '<other_task-request>"
  (cl:let* ((signed (cl:slot-value msg 'task_data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <other_task-request>) istream)
  "Deserializes a message object of type '<other_task-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task_data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<other_task-request>)))
  "Returns string type for a service object of type '<other_task-request>"
  "robot_sim/other_taskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'other_task-request)))
  "Returns string type for a service object of type 'other_task-request"
  "robot_sim/other_taskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<other_task-request>)))
  "Returns md5sum for a message object of type '<other_task-request>"
  "d0f539cb3e5e23bc53002dc7b1f24555")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'other_task-request)))
  "Returns md5sum for a message object of type 'other_task-request"
  "d0f539cb3e5e23bc53002dc7b1f24555")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<other_task-request>)))
  "Returns full string definition for message of type '<other_task-request>"
  (cl:format cl:nil "int32 task_data~%string task_type~%int64 task_id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'other_task-request)))
  "Returns full string definition for message of type 'other_task-request"
  (cl:format cl:nil "int32 task_data~%string task_type~%int64 task_id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <other_task-request>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'task_type))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <other_task-request>))
  "Converts a ROS message object to a list"
  (cl:list 'other_task-request
    (cl:cons ':task_data (task_data msg))
    (cl:cons ':task_type (task_type msg))
    (cl:cons ':task_id (task_id msg))
))
;//! \htmlinclude other_task-response.msg.html

(cl:defclass <other_task-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass other_task-response (<other_task-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <other_task-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'other_task-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_sim-srv:<other_task-response> is deprecated: use robot_sim-srv:other_task-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <other_task-response>) ostream)
  "Serializes a message object of type '<other_task-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <other_task-response>) istream)
  "Deserializes a message object of type '<other_task-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<other_task-response>)))
  "Returns string type for a service object of type '<other_task-response>"
  "robot_sim/other_taskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'other_task-response)))
  "Returns string type for a service object of type 'other_task-response"
  "robot_sim/other_taskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<other_task-response>)))
  "Returns md5sum for a message object of type '<other_task-response>"
  "d0f539cb3e5e23bc53002dc7b1f24555")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'other_task-response)))
  "Returns md5sum for a message object of type 'other_task-response"
  "d0f539cb3e5e23bc53002dc7b1f24555")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<other_task-response>)))
  "Returns full string definition for message of type '<other_task-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'other_task-response)))
  "Returns full string definition for message of type 'other_task-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <other_task-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <other_task-response>))
  "Converts a ROS message object to a list"
  (cl:list 'other_task-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'other_task)))
  'other_task-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'other_task)))
  'other_task-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'other_task)))
  "Returns string type for a service object of type '<other_task>"
  "robot_sim/other_task")