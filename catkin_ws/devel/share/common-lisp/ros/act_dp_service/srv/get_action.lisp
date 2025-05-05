; Auto-generated. Do not edit!


(cl:in-package act_dp_service-srv)


;//! \htmlinclude get_action-request.msg.html

(cl:defclass <get_action-request> (roslisp-msg-protocol:ros-message)
  ((sensor
    :reader sensor
    :initarg :sensor
    :type act_dp_service-msg:RawData
    :initform (cl:make-instance 'act_dp_service-msg:RawData)))
)

(cl:defclass get_action-request (<get_action-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_action-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_action-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name act_dp_service-srv:<get_action-request> is deprecated: use act_dp_service-srv:get_action-request instead.")))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <get_action-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_dp_service-srv:sensor-val is deprecated.  Use act_dp_service-srv:sensor instead.")
  (sensor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_action-request>) ostream)
  "Serializes a message object of type '<get_action-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sensor) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_action-request>) istream)
  "Deserializes a message object of type '<get_action-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sensor) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_action-request>)))
  "Returns string type for a service object of type '<get_action-request>"
  "act_dp_service/get_actionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_action-request)))
  "Returns string type for a service object of type 'get_action-request"
  "act_dp_service/get_actionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_action-request>)))
  "Returns md5sum for a message object of type '<get_action-request>"
  "158a528fc884a7668a43b64fd5398f56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_action-request)))
  "Returns md5sum for a message object of type 'get_action-request"
  "158a528fc884a7668a43b64fd5398f56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_action-request>)))
  "Returns full string definition for message of type '<get_action-request>"
  (cl:format cl:nil "# 输入: 传感器数据~%RawData sensor~%~%~%================================================================================~%MSG: act_dp_service/RawData~%std_msgs/Float64MultiArray ee_pose     # 当前末端执行器 (End-Effector) 的位姿信息, 数据格式为一维数组: [位置 x, 位置 y, 位置 z, 朝向四元数 w, 朝向四元数 x, 朝向四元数 y, 朝向四元数 z]~%std_msgs/Float64MultiArray joint_pos     # 关节角度，[joint1,joint2,...,joint7]~%~%std_msgs/Float64MultiArray init_ee_pose    # 初始末端执行器 (End-Effector) 的位姿信息~%std_msgs/Float64MultiArray init_joint_pos    # 初始关节角度~%~%std_msgs/Float64 gripper_width           # 夹爪宽度~%std_msgs/UInt8MultiArray rgb_data        # RGB 图像数据， (height x width x 3)~%~%std_msgs/Bool            reset           # 重置信号：True 表示请求系统重置或回到初始状态~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/UInt8MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint8[]           data          # array of data~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_action-request)))
  "Returns full string definition for message of type 'get_action-request"
  (cl:format cl:nil "# 输入: 传感器数据~%RawData sensor~%~%~%================================================================================~%MSG: act_dp_service/RawData~%std_msgs/Float64MultiArray ee_pose     # 当前末端执行器 (End-Effector) 的位姿信息, 数据格式为一维数组: [位置 x, 位置 y, 位置 z, 朝向四元数 w, 朝向四元数 x, 朝向四元数 y, 朝向四元数 z]~%std_msgs/Float64MultiArray joint_pos     # 关节角度，[joint1,joint2,...,joint7]~%~%std_msgs/Float64MultiArray init_ee_pose    # 初始末端执行器 (End-Effector) 的位姿信息~%std_msgs/Float64MultiArray init_joint_pos    # 初始关节角度~%~%std_msgs/Float64 gripper_width           # 夹爪宽度~%std_msgs/UInt8MultiArray rgb_data        # RGB 图像数据， (height x width x 3)~%~%std_msgs/Bool            reset           # 重置信号：True 表示请求系统重置或回到初始状态~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/UInt8MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint8[]           data          # array of data~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_action-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sensor))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_action-request>))
  "Converts a ROS message object to a list"
  (cl:list 'get_action-request
    (cl:cons ':sensor (sensor msg))
))
;//! \htmlinclude get_action-response.msg.html

(cl:defclass <get_action-response> (roslisp-msg-protocol:ros-message)
  ((actions
    :reader actions
    :initarg :actions
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray)))
)

(cl:defclass get_action-response (<get_action-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_action-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_action-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name act_dp_service-srv:<get_action-response> is deprecated: use act_dp_service-srv:get_action-response instead.")))

(cl:ensure-generic-function 'actions-val :lambda-list '(m))
(cl:defmethod actions-val ((m <get_action-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_dp_service-srv:actions-val is deprecated.  Use act_dp_service-srv:actions instead.")
  (actions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_action-response>) ostream)
  "Serializes a message object of type '<get_action-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'actions) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_action-response>) istream)
  "Deserializes a message object of type '<get_action-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'actions) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_action-response>)))
  "Returns string type for a service object of type '<get_action-response>"
  "act_dp_service/get_actionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_action-response)))
  "Returns string type for a service object of type 'get_action-response"
  "act_dp_service/get_actionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_action-response>)))
  "Returns md5sum for a message object of type '<get_action-response>"
  "158a528fc884a7668a43b64fd5398f56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_action-response)))
  "Returns md5sum for a message object of type 'get_action-response"
  "158a528fc884a7668a43b64fd5398f56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_action-response>)))
  "Returns full string definition for message of type '<get_action-response>"
  (cl:format cl:nil "# 输出: 机器人关节角度（numpy 数组）~%std_msgs/Float64MultiArray actions~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_action-response)))
  "Returns full string definition for message of type 'get_action-response"
  (cl:format cl:nil "# 输出: 机器人关节角度（numpy 数组）~%std_msgs/Float64MultiArray actions~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_action-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'actions))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_action-response>))
  "Converts a ROS message object to a list"
  (cl:list 'get_action-response
    (cl:cons ':actions (actions msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'get_action)))
  'get_action-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'get_action)))
  'get_action-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_action)))
  "Returns string type for a service object of type '<get_action>"
  "act_dp_service/get_action")