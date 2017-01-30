; Auto-generated. Do not edit!


(cl:in-package Human_intention-msg)


;//! \htmlinclude EnableJointGroup.msg.html

(cl:defclass <EnableJointGroup> (roslisp-msg-protocol:ros-message)
  ((jointGroups
    :reader jointGroups
    :initarg :jointGroups
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (enabledStates
    :reader enabledStates
    :initarg :enabledStates
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass EnableJointGroup (<EnableJointGroup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableJointGroup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableJointGroup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Human_intention-msg:<EnableJointGroup> is deprecated: use Human_intention-msg:EnableJointGroup instead.")))

(cl:ensure-generic-function 'jointGroups-val :lambda-list '(m))
(cl:defmethod jointGroups-val ((m <EnableJointGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Human_intention-msg:jointGroups-val is deprecated.  Use Human_intention-msg:jointGroups instead.")
  (jointGroups m))

(cl:ensure-generic-function 'enabledStates-val :lambda-list '(m))
(cl:defmethod enabledStates-val ((m <EnableJointGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Human_intention-msg:enabledStates-val is deprecated.  Use Human_intention-msg:enabledStates instead.")
  (enabledStates m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableJointGroup>) ostream)
  "Serializes a message object of type '<EnableJointGroup>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'jointGroups))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'jointGroups))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'enabledStates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'enabledStates))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableJointGroup>) istream)
  "Deserializes a message object of type '<EnableJointGroup>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'jointGroups) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'jointGroups)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'enabledStates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'enabledStates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableJointGroup>)))
  "Returns string type for a message object of type '<EnableJointGroup>"
  "Human_intention/EnableJointGroup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableJointGroup)))
  "Returns string type for a message object of type 'EnableJointGroup"
  "Human_intention/EnableJointGroup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableJointGroup>)))
  "Returns md5sum for a message object of type '<EnableJointGroup>"
  "816ee069696c3513a51b0f478a453767")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableJointGroup)))
  "Returns md5sum for a message object of type 'EnableJointGroup"
  "816ee069696c3513a51b0f478a453767")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableJointGroup>)))
  "Returns full string definition for message of type '<EnableJointGroup>"
  (cl:format cl:nil "string[] jointGroups~%bool[] enabledStates~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableJointGroup)))
  "Returns full string definition for message of type 'EnableJointGroup"
  (cl:format cl:nil "string[] jointGroups~%bool[] enabledStates~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableJointGroup>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'jointGroups) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'enabledStates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableJointGroup>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableJointGroup
    (cl:cons ':jointGroups (jointGroups msg))
    (cl:cons ':enabledStates (enabledStates msg))
))
