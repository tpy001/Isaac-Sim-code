;; Auto-generated. Do not edit!


(when (boundp 'act_dp_service::get_action)
  (if (not (find-package "ACT_DP_SERVICE"))
    (make-package "ACT_DP_SERVICE"))
  (shadow 'get_action (find-package "ACT_DP_SERVICE")))
(unless (find-package "ACT_DP_SERVICE::GET_ACTION")
  (make-package "ACT_DP_SERVICE::GET_ACTION"))
(unless (find-package "ACT_DP_SERVICE::GET_ACTIONREQUEST")
  (make-package "ACT_DP_SERVICE::GET_ACTIONREQUEST"))
(unless (find-package "ACT_DP_SERVICE::GET_ACTIONRESPONSE")
  (make-package "ACT_DP_SERVICE::GET_ACTIONRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass act_dp_service::get_actionRequest
  :super ros::object
  :slots (_sensor ))

(defmethod act_dp_service::get_actionRequest
  (:init
   (&key
    ((:sensor __sensor) (instance act_dp_service::RawData :init))
    )
   (send-super :init)
   (setq _sensor __sensor)
   self)
  (:sensor
   (&rest __sensor)
   (if (keywordp (car __sensor))
       (send* _sensor __sensor)
     (progn
       (if __sensor (setq _sensor (car __sensor)))
       _sensor)))
  (:serialization-length
   ()
   (+
    ;; act_dp_service/RawData _sensor
    (send _sensor :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; act_dp_service/RawData _sensor
       (send _sensor :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; act_dp_service/RawData _sensor
     (send _sensor :deserialize buf ptr-) (incf ptr- (send _sensor :serialization-length))
   ;;
   self)
  )

(defclass act_dp_service::get_actionResponse
  :super ros::object
  :slots (_actions ))

(defmethod act_dp_service::get_actionResponse
  (:init
   (&key
    ((:actions __actions) (instance std_msgs::Float64MultiArray :init))
    )
   (send-super :init)
   (setq _actions __actions)
   self)
  (:actions
   (&rest __actions)
   (if (keywordp (car __actions))
       (send* _actions __actions)
     (progn
       (if __actions (setq _actions (car __actions)))
       _actions)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray _actions
    (send _actions :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64MultiArray _actions
       (send _actions :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64MultiArray _actions
     (send _actions :deserialize buf ptr-) (incf ptr- (send _actions :serialization-length))
   ;;
   self)
  )

(defclass act_dp_service::get_action
  :super ros::object
  :slots ())

(setf (get act_dp_service::get_action :md5sum-) "bb5933a9fa74805641f297a01d40ec1d")
(setf (get act_dp_service::get_action :datatype-) "act_dp_service/get_action")
(setf (get act_dp_service::get_action :request) act_dp_service::get_actionRequest)
(setf (get act_dp_service::get_action :response) act_dp_service::get_actionResponse)

(defmethod act_dp_service::get_actionRequest
  (:response () (instance act_dp_service::get_actionResponse :init)))

(setf (get act_dp_service::get_actionRequest :md5sum-) "bb5933a9fa74805641f297a01d40ec1d")
(setf (get act_dp_service::get_actionRequest :datatype-) "act_dp_service/get_actionRequest")
(setf (get act_dp_service::get_actionRequest :definition-)
      "# 输入: 传感器数据
RawData sensor


================================================================================
MSG: act_dp_service/RawData
std_msgs/Float64MultiArray ee_pose     # 当前末端执行器 (End-Effector) 的位姿信息, 数据格式为一维数组: [位置 x, 位置 y, 位置 z, 朝向四元数 w, 朝向四元数 x, 朝向四元数 y, 朝向四元数 z]
std_msgs/Float64MultiArray joint_pos     # 关节角度，[joint1,joint2,...,joint7]

std_msgs/Float64MultiArray init_ee_pose    # 初始末端执行器 (End-Effector) 的位姿信息
std_msgs/Float64MultiArray init_joint_pos    # 初始关节角度

std_msgs/Float64 gripper_width           # 夹爪宽度
std_msgs/UInt8MultiArray rgb_data        # RGB 图像数据， (height x width x 3)
std_msgs/UInt8MultiArray depth_data        # 深度图 colorful (height x width x 3)

std_msgs/Bool            reset           # 重置信号：True 表示请求系统重置或回到初始状态
================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: std_msgs/UInt8MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint8[]           data          # array of data


================================================================================
MSG: std_msgs/Bool
---
# 输出: 机器人关节角度（numpy 数组）
std_msgs/Float64MultiArray actions

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")

(setf (get act_dp_service::get_actionResponse :md5sum-) "bb5933a9fa74805641f297a01d40ec1d")
(setf (get act_dp_service::get_actionResponse :datatype-) "act_dp_service/get_actionResponse")
(setf (get act_dp_service::get_actionResponse :definition-)
      "# 输入: 传感器数据
RawData sensor


================================================================================
MSG: act_dp_service/RawData
std_msgs/Float64MultiArray ee_pose     # 当前末端执行器 (End-Effector) 的位姿信息, 数据格式为一维数组: [位置 x, 位置 y, 位置 z, 朝向四元数 w, 朝向四元数 x, 朝向四元数 y, 朝向四元数 z]
std_msgs/Float64MultiArray joint_pos     # 关节角度，[joint1,joint2,...,joint7]

std_msgs/Float64MultiArray init_ee_pose    # 初始末端执行器 (End-Effector) 的位姿信息
std_msgs/Float64MultiArray init_joint_pos    # 初始关节角度

std_msgs/Float64 gripper_width           # 夹爪宽度
std_msgs/UInt8MultiArray rgb_data        # RGB 图像数据， (height x width x 3)
std_msgs/UInt8MultiArray depth_data        # 深度图 colorful (height x width x 3)

std_msgs/Bool            reset           # 重置信号：True 表示请求系统重置或回到初始状态
================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: std_msgs/UInt8MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint8[]           data          # array of data


================================================================================
MSG: std_msgs/Bool
---
# 输出: 机器人关节角度（numpy 数组）
std_msgs/Float64MultiArray actions

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")



(provide :act_dp_service/get_action "bb5933a9fa74805641f297a01d40ec1d")


