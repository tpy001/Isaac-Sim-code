;; Auto-generated. Do not edit!


(when (boundp 'act_dp_service::RawData)
  (if (not (find-package "ACT_DP_SERVICE"))
    (make-package "ACT_DP_SERVICE"))
  (shadow 'RawData (find-package "ACT_DP_SERVICE")))
(unless (find-package "ACT_DP_SERVICE::RAWDATA")
  (make-package "ACT_DP_SERVICE::RAWDATA"))

(in-package "ROS")
;;//! \htmlinclude RawData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass act_dp_service::RawData
  :super ros::object
  :slots (_ee_pose _joint_pos _init_ee_pose _init_joint_pos _gripper_width _rgb_data _depth_data _reset ))

(defmethod act_dp_service::RawData
  (:init
   (&key
    ((:ee_pose __ee_pose) (instance std_msgs::Float64MultiArray :init))
    ((:joint_pos __joint_pos) (instance std_msgs::Float64MultiArray :init))
    ((:init_ee_pose __init_ee_pose) (instance std_msgs::Float64MultiArray :init))
    ((:init_joint_pos __init_joint_pos) (instance std_msgs::Float64MultiArray :init))
    ((:gripper_width __gripper_width) (instance std_msgs::Float64 :init))
    ((:rgb_data __rgb_data) (instance std_msgs::UInt8MultiArray :init))
    ((:depth_data __depth_data) (instance std_msgs::UInt8MultiArray :init))
    ((:reset __reset) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _ee_pose __ee_pose)
   (setq _joint_pos __joint_pos)
   (setq _init_ee_pose __init_ee_pose)
   (setq _init_joint_pos __init_joint_pos)
   (setq _gripper_width __gripper_width)
   (setq _rgb_data __rgb_data)
   (setq _depth_data __depth_data)
   (setq _reset __reset)
   self)
  (:ee_pose
   (&rest __ee_pose)
   (if (keywordp (car __ee_pose))
       (send* _ee_pose __ee_pose)
     (progn
       (if __ee_pose (setq _ee_pose (car __ee_pose)))
       _ee_pose)))
  (:joint_pos
   (&rest __joint_pos)
   (if (keywordp (car __joint_pos))
       (send* _joint_pos __joint_pos)
     (progn
       (if __joint_pos (setq _joint_pos (car __joint_pos)))
       _joint_pos)))
  (:init_ee_pose
   (&rest __init_ee_pose)
   (if (keywordp (car __init_ee_pose))
       (send* _init_ee_pose __init_ee_pose)
     (progn
       (if __init_ee_pose (setq _init_ee_pose (car __init_ee_pose)))
       _init_ee_pose)))
  (:init_joint_pos
   (&rest __init_joint_pos)
   (if (keywordp (car __init_joint_pos))
       (send* _init_joint_pos __init_joint_pos)
     (progn
       (if __init_joint_pos (setq _init_joint_pos (car __init_joint_pos)))
       _init_joint_pos)))
  (:gripper_width
   (&rest __gripper_width)
   (if (keywordp (car __gripper_width))
       (send* _gripper_width __gripper_width)
     (progn
       (if __gripper_width (setq _gripper_width (car __gripper_width)))
       _gripper_width)))
  (:rgb_data
   (&rest __rgb_data)
   (if (keywordp (car __rgb_data))
       (send* _rgb_data __rgb_data)
     (progn
       (if __rgb_data (setq _rgb_data (car __rgb_data)))
       _rgb_data)))
  (:depth_data
   (&rest __depth_data)
   (if (keywordp (car __depth_data))
       (send* _depth_data __depth_data)
     (progn
       (if __depth_data (setq _depth_data (car __depth_data)))
       _depth_data)))
  (:reset
   (&rest __reset)
   (if (keywordp (car __reset))
       (send* _reset __reset)
     (progn
       (if __reset (setq _reset (car __reset)))
       _reset)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray _ee_pose
    (send _ee_pose :serialization-length)
    ;; std_msgs/Float64MultiArray _joint_pos
    (send _joint_pos :serialization-length)
    ;; std_msgs/Float64MultiArray _init_ee_pose
    (send _init_ee_pose :serialization-length)
    ;; std_msgs/Float64MultiArray _init_joint_pos
    (send _init_joint_pos :serialization-length)
    ;; std_msgs/Float64 _gripper_width
    (send _gripper_width :serialization-length)
    ;; std_msgs/UInt8MultiArray _rgb_data
    (send _rgb_data :serialization-length)
    ;; std_msgs/UInt8MultiArray _depth_data
    (send _depth_data :serialization-length)
    ;; std_msgs/Bool _reset
    (send _reset :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64MultiArray _ee_pose
       (send _ee_pose :serialize s)
     ;; std_msgs/Float64MultiArray _joint_pos
       (send _joint_pos :serialize s)
     ;; std_msgs/Float64MultiArray _init_ee_pose
       (send _init_ee_pose :serialize s)
     ;; std_msgs/Float64MultiArray _init_joint_pos
       (send _init_joint_pos :serialize s)
     ;; std_msgs/Float64 _gripper_width
       (send _gripper_width :serialize s)
     ;; std_msgs/UInt8MultiArray _rgb_data
       (send _rgb_data :serialize s)
     ;; std_msgs/UInt8MultiArray _depth_data
       (send _depth_data :serialize s)
     ;; std_msgs/Bool _reset
       (send _reset :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64MultiArray _ee_pose
     (send _ee_pose :deserialize buf ptr-) (incf ptr- (send _ee_pose :serialization-length))
   ;; std_msgs/Float64MultiArray _joint_pos
     (send _joint_pos :deserialize buf ptr-) (incf ptr- (send _joint_pos :serialization-length))
   ;; std_msgs/Float64MultiArray _init_ee_pose
     (send _init_ee_pose :deserialize buf ptr-) (incf ptr- (send _init_ee_pose :serialization-length))
   ;; std_msgs/Float64MultiArray _init_joint_pos
     (send _init_joint_pos :deserialize buf ptr-) (incf ptr- (send _init_joint_pos :serialization-length))
   ;; std_msgs/Float64 _gripper_width
     (send _gripper_width :deserialize buf ptr-) (incf ptr- (send _gripper_width :serialization-length))
   ;; std_msgs/UInt8MultiArray _rgb_data
     (send _rgb_data :deserialize buf ptr-) (incf ptr- (send _rgb_data :serialization-length))
   ;; std_msgs/UInt8MultiArray _depth_data
     (send _depth_data :deserialize buf ptr-) (incf ptr- (send _depth_data :serialization-length))
   ;; std_msgs/Bool _reset
     (send _reset :deserialize buf ptr-) (incf ptr- (send _reset :serialization-length))
   ;;
   self)
  )

(setf (get act_dp_service::RawData :md5sum-) "2028a2619c88661ed127cab0fd5a32f5")
(setf (get act_dp_service::RawData :datatype-) "act_dp_service/RawData")
(setf (get act_dp_service::RawData :definition-)
      "std_msgs/Float64MultiArray ee_pose     # 当前末端执行器 (End-Effector) 的位姿信息, 数据格式为一维数组: [位置 x, 位置 y, 位置 z, 朝向四元数 w, 朝向四元数 x, 朝向四元数 y, 朝向四元数 z]
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
bool data
")



(provide :act_dp_service/RawData "2028a2619c88661ed127cab0fd5a32f5")


