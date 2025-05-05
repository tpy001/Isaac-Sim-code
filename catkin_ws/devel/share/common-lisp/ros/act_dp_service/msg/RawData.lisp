; Auto-generated. Do not edit!


(cl:in-package act_dp_service-msg)


;//! \htmlinclude RawData.msg.html

(cl:defclass <RawData> (roslisp-msg-protocol:ros-message)
  ((ee_pose
    :reader ee_pose
    :initarg :ee_pose
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (joint_pos
    :reader joint_pos
    :initarg :joint_pos
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (init_ee_pose
    :reader init_ee_pose
    :initarg :init_ee_pose
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (init_joint_pos
    :reader init_joint_pos
    :initarg :init_joint_pos
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (gripper_width
    :reader gripper_width
    :initarg :gripper_width
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (rgb_data
    :reader rgb_data
    :initarg :rgb_data
    :type std_msgs-msg:UInt8MultiArray
    :initform (cl:make-instance 'std_msgs-msg:UInt8MultiArray))
   (reset
    :reader reset
    :initarg :reset
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass RawData (<RawData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name act_dp_service-msg:<RawData> is deprecated: use act_dp_service-msg:RawData instead.")))

(cl:ensure-generic-function 'ee_pose-val :lambda-list '(m))
(cl:defmethod ee_pose-val ((m <RawData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_dp_service-msg:ee_pose-val is deprecated.  Use act_dp_service-msg:ee_pose instead.")
  (ee_pose m))

(cl:ensure-generic-function 'joint_pos-val :lambda-list '(m))
(cl:defmethod joint_pos-val ((m <RawData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_dp_service-msg:joint_pos-val is deprecated.  Use act_dp_service-msg:joint_pos instead.")
  (joint_pos m))

(cl:ensure-generic-function 'init_ee_pose-val :lambda-list '(m))
(cl:defmethod init_ee_pose-val ((m <RawData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_dp_service-msg:init_ee_pose-val is deprecated.  Use act_dp_service-msg:init_ee_pose instead.")
  (init_ee_pose m))

(cl:ensure-generic-function 'init_joint_pos-val :lambda-list '(m))
(cl:defmethod init_joint_pos-val ((m <RawData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_dp_service-msg:init_joint_pos-val is deprecated.  Use act_dp_service-msg:init_joint_pos instead.")
  (init_joint_pos m))

(cl:ensure-generic-function 'gripper_width-val :lambda-list '(m))
(cl:defmethod gripper_width-val ((m <RawData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_dp_service-msg:gripper_width-val is deprecated.  Use act_dp_service-msg:gripper_width instead.")
  (gripper_width m))

(cl:ensure-generic-function 'rgb_data-val :lambda-list '(m))
(cl:defmethod rgb_data-val ((m <RawData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_dp_service-msg:rgb_data-val is deprecated.  Use act_dp_service-msg:rgb_data instead.")
  (rgb_data m))

(cl:ensure-generic-function 'reset-val :lambda-list '(m))
(cl:defmethod reset-val ((m <RawData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_dp_service-msg:reset-val is deprecated.  Use act_dp_service-msg:reset instead.")
  (reset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawData>) ostream)
  "Serializes a message object of type '<RawData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ee_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'init_ee_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'init_joint_pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gripper_width) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rgb_data) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'reset) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawData>) istream)
  "Deserializes a message object of type '<RawData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ee_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'init_ee_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'init_joint_pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gripper_width) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rgb_data) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'reset) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawData>)))
  "Returns string type for a message object of type '<RawData>"
  "act_dp_service/RawData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawData)))
  "Returns string type for a message object of type 'RawData"
  "act_dp_service/RawData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawData>)))
  "Returns md5sum for a message object of type '<RawData>"
  "a2490b5f4d8422f6e431911c53295728")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawData)))
  "Returns md5sum for a message object of type 'RawData"
  "a2490b5f4d8422f6e431911c53295728")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawData>)))
  "Returns full string definition for message of type '<RawData>"
  (cl:format cl:nil "std_msgs/Float64MultiArray ee_pose     # 当前末端执行器 (End-Effector) 的位姿信息, 数据格式为一维数组: [位置 x, 位置 y, 位置 z, 朝向四元数 w, 朝向四元数 x, 朝向四元数 y, 朝向四元数 z]~%std_msgs/Float64MultiArray joint_pos     # 关节角度，[joint1,joint2,...,joint7]~%~%std_msgs/Float64MultiArray init_ee_pose    # 初始末端执行器 (End-Effector) 的位姿信息~%std_msgs/Float64MultiArray init_joint_pos    # 初始关节角度~%~%std_msgs/Float64 gripper_width           # 夹爪宽度~%std_msgs/UInt8MultiArray rgb_data        # RGB 图像数据， (height x width x 3)~%~%std_msgs/Bool            reset           # 重置信号：True 表示请求系统重置或回到初始状态~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/UInt8MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint8[]           data          # array of data~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawData)))
  "Returns full string definition for message of type 'RawData"
  (cl:format cl:nil "std_msgs/Float64MultiArray ee_pose     # 当前末端执行器 (End-Effector) 的位姿信息, 数据格式为一维数组: [位置 x, 位置 y, 位置 z, 朝向四元数 w, 朝向四元数 x, 朝向四元数 y, 朝向四元数 z]~%std_msgs/Float64MultiArray joint_pos     # 关节角度，[joint1,joint2,...,joint7]~%~%std_msgs/Float64MultiArray init_ee_pose    # 初始末端执行器 (End-Effector) 的位姿信息~%std_msgs/Float64MultiArray init_joint_pos    # 初始关节角度~%~%std_msgs/Float64 gripper_width           # 夹爪宽度~%std_msgs/UInt8MultiArray rgb_data        # RGB 图像数据， (height x width x 3)~%~%std_msgs/Bool            reset           # 重置信号：True 表示请求系统重置或回到初始状态~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/UInt8MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint8[]           data          # array of data~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ee_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'init_ee_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'init_joint_pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gripper_width))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rgb_data))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'reset))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawData>))
  "Converts a ROS message object to a list"
  (cl:list 'RawData
    (cl:cons ':ee_pose (ee_pose msg))
    (cl:cons ':joint_pos (joint_pos msg))
    (cl:cons ':init_ee_pose (init_ee_pose msg))
    (cl:cons ':init_joint_pos (init_joint_pos msg))
    (cl:cons ':gripper_width (gripper_width msg))
    (cl:cons ':rgb_data (rgb_data msg))
    (cl:cons ':reset (reset msg))
))
