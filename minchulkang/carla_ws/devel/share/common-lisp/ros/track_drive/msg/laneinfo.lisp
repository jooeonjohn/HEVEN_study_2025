; Auto-generated. Do not edit!


(cl:in-package track_drive-msg)


;//! \htmlinclude laneinfo.msg.html

(cl:defclass <laneinfo> (roslisp-msg-protocol:ros-message)
  ((left_x
    :reader left_x
    :initarg :left_x
    :type cl:float
    :initform 0.0)
   (left_y
    :reader left_y
    :initarg :left_y
    :type cl:float
    :initform 0.0)
   (left_slope
    :reader left_slope
    :initarg :left_slope
    :type cl:float
    :initform 0.0)
   (right_x
    :reader right_x
    :initarg :right_x
    :type cl:float
    :initform 0.0)
   (right_y
    :reader right_y
    :initarg :right_y
    :type cl:float
    :initform 0.0)
   (right_slope
    :reader right_slope
    :initarg :right_slope
    :type cl:float
    :initform 0.0))
)

(cl:defclass laneinfo (<laneinfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <laneinfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'laneinfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name track_drive-msg:<laneinfo> is deprecated: use track_drive-msg:laneinfo instead.")))

(cl:ensure-generic-function 'left_x-val :lambda-list '(m))
(cl:defmethod left_x-val ((m <laneinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_drive-msg:left_x-val is deprecated.  Use track_drive-msg:left_x instead.")
  (left_x m))

(cl:ensure-generic-function 'left_y-val :lambda-list '(m))
(cl:defmethod left_y-val ((m <laneinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_drive-msg:left_y-val is deprecated.  Use track_drive-msg:left_y instead.")
  (left_y m))

(cl:ensure-generic-function 'left_slope-val :lambda-list '(m))
(cl:defmethod left_slope-val ((m <laneinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_drive-msg:left_slope-val is deprecated.  Use track_drive-msg:left_slope instead.")
  (left_slope m))

(cl:ensure-generic-function 'right_x-val :lambda-list '(m))
(cl:defmethod right_x-val ((m <laneinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_drive-msg:right_x-val is deprecated.  Use track_drive-msg:right_x instead.")
  (right_x m))

(cl:ensure-generic-function 'right_y-val :lambda-list '(m))
(cl:defmethod right_y-val ((m <laneinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_drive-msg:right_y-val is deprecated.  Use track_drive-msg:right_y instead.")
  (right_y m))

(cl:ensure-generic-function 'right_slope-val :lambda-list '(m))
(cl:defmethod right_slope-val ((m <laneinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_drive-msg:right_slope-val is deprecated.  Use track_drive-msg:right_slope instead.")
  (right_slope m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <laneinfo>) ostream)
  "Serializes a message object of type '<laneinfo>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_slope))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_slope))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <laneinfo>) istream)
  "Deserializes a message object of type '<laneinfo>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_slope) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_slope) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<laneinfo>)))
  "Returns string type for a message object of type '<laneinfo>"
  "track_drive/laneinfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'laneinfo)))
  "Returns string type for a message object of type 'laneinfo"
  "track_drive/laneinfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<laneinfo>)))
  "Returns md5sum for a message object of type '<laneinfo>"
  "44a42243e734addbc7ddd0e00184e0ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'laneinfo)))
  "Returns md5sum for a message object of type 'laneinfo"
  "44a42243e734addbc7ddd0e00184e0ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<laneinfo>)))
  "Returns full string definition for message of type '<laneinfo>"
  (cl:format cl:nil "float32 left_x~%float32 left_y~%float32 left_slope~%float32 right_x~%float32 right_y~%float32 right_slope~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'laneinfo)))
  "Returns full string definition for message of type 'laneinfo"
  (cl:format cl:nil "float32 left_x~%float32 left_y~%float32 left_slope~%float32 right_x~%float32 right_y~%float32 right_slope~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <laneinfo>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <laneinfo>))
  "Converts a ROS message object to a list"
  (cl:list 'laneinfo
    (cl:cons ':left_x (left_x msg))
    (cl:cons ':left_y (left_y msg))
    (cl:cons ':left_slope (left_slope msg))
    (cl:cons ':right_x (right_x msg))
    (cl:cons ':right_y (right_y msg))
    (cl:cons ':right_slope (right_slope msg))
))
