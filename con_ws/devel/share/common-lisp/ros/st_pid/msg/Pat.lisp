; Auto-generated. Do not edit!


(cl:in-package st_pid-msg)


;//! \htmlinclude Pat.msg.html

(cl:defclass <Pat> (roslisp-msg-protocol:ros-message)
  ((path_
    :reader path_
    :initarg :path_
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Pat (<Pat>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pat>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pat)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name st_pid-msg:<Pat> is deprecated: use st_pid-msg:Pat instead.")))

(cl:ensure-generic-function 'path_-val :lambda-list '(m))
(cl:defmethod path_-val ((m <Pat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader st_pid-msg:path_-val is deprecated.  Use st_pid-msg:path_ instead.")
  (path_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pat>) ostream)
  "Serializes a message object of type '<Pat>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path_))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'path_))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pat>) istream)
  "Deserializes a message object of type '<Pat>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path_) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path_)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pat>)))
  "Returns string type for a message object of type '<Pat>"
  "st_pid/Pat")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pat)))
  "Returns string type for a message object of type 'Pat"
  "st_pid/Pat")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pat>)))
  "Returns md5sum for a message object of type '<Pat>"
  "7e7e4cb758a1a3e10b6dab1976b351ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pat)))
  "Returns md5sum for a message object of type 'Pat"
  "7e7e4cb758a1a3e10b6dab1976b351ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pat>)))
  "Returns full string definition for message of type '<Pat>"
  (cl:format cl:nil "uint8[] path_~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pat)))
  "Returns full string definition for message of type 'Pat"
  (cl:format cl:nil "uint8[] path_~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pat>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path_) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pat>))
  "Converts a ROS message object to a list"
  (cl:list 'Pat
    (cl:cons ':path_ (path_ msg))
))
