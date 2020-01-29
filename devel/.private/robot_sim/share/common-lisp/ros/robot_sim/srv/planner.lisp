; Auto-generated. Do not edit!


(cl:in-package robot_sim-srv)


;//! \htmlinclude planner-request.msg.html

(cl:defclass <planner-request> (roslisp-msg-protocol:ros-message)
  ((coords_msg
    :reader coords_msg
    :initarg :coords_msg
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass planner-request (<planner-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <planner-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'planner-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_sim-srv:<planner-request> is deprecated: use robot_sim-srv:planner-request instead.")))

(cl:ensure-generic-function 'coords_msg-val :lambda-list '(m))
(cl:defmethod coords_msg-val ((m <planner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_sim-srv:coords_msg-val is deprecated.  Use robot_sim-srv:coords_msg instead.")
  (coords_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <planner-request>) ostream)
  "Serializes a message object of type '<planner-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coords_msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'coords_msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <planner-request>) istream)
  "Deserializes a message object of type '<planner-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coords_msg) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coords_msg)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<planner-request>)))
  "Returns string type for a service object of type '<planner-request>"
  "robot_sim/plannerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'planner-request)))
  "Returns string type for a service object of type 'planner-request"
  "robot_sim/plannerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<planner-request>)))
  "Returns md5sum for a message object of type '<planner-request>"
  "6423f791599f0a66e62771c89cda1dcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'planner-request)))
  "Returns md5sum for a message object of type 'planner-request"
  "6423f791599f0a66e62771c89cda1dcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<planner-request>)))
  "Returns full string definition for message of type '<planner-request>"
  (cl:format cl:nil "float32[] coords_msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'planner-request)))
  "Returns full string definition for message of type 'planner-request"
  (cl:format cl:nil "float32[] coords_msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <planner-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coords_msg) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <planner-request>))
  "Converts a ROS message object to a list"
  (cl:list 'planner-request
    (cl:cons ':coords_msg (coords_msg msg))
))
;//! \htmlinclude planner-response.msg.html

(cl:defclass <planner-response> (roslisp-msg-protocol:ros-message)
  ((optimal_path
    :reader optimal_path
    :initarg :optimal_path
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass planner-response (<planner-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <planner-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'planner-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_sim-srv:<planner-response> is deprecated: use robot_sim-srv:planner-response instead.")))

(cl:ensure-generic-function 'optimal_path-val :lambda-list '(m))
(cl:defmethod optimal_path-val ((m <planner-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_sim-srv:optimal_path-val is deprecated.  Use robot_sim-srv:optimal_path instead.")
  (optimal_path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <planner-response>) ostream)
  "Serializes a message object of type '<planner-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'optimal_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'optimal_path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <planner-response>) istream)
  "Deserializes a message object of type '<planner-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'optimal_path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'optimal_path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<planner-response>)))
  "Returns string type for a service object of type '<planner-response>"
  "robot_sim/plannerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'planner-response)))
  "Returns string type for a service object of type 'planner-response"
  "robot_sim/plannerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<planner-response>)))
  "Returns md5sum for a message object of type '<planner-response>"
  "6423f791599f0a66e62771c89cda1dcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'planner-response)))
  "Returns md5sum for a message object of type 'planner-response"
  "6423f791599f0a66e62771c89cda1dcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<planner-response>)))
  "Returns full string definition for message of type '<planner-response>"
  (cl:format cl:nil "float32[] optimal_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'planner-response)))
  "Returns full string definition for message of type 'planner-response"
  (cl:format cl:nil "float32[] optimal_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <planner-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'optimal_path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <planner-response>))
  "Converts a ROS message object to a list"
  (cl:list 'planner-response
    (cl:cons ':optimal_path (optimal_path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'planner)))
  'planner-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'planner)))
  'planner-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'planner)))
  "Returns string type for a service object of type '<planner>"
  "robot_sim/planner")