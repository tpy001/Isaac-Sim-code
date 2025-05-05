std_msgs/Float64MultiArray ee_pose     # 当前末端执行器 (End-Effector) 的位姿信息, 数据格式为一维数组: [位置 x, 位置 y, 位置 z, 朝向四元数 w, 朝向四元数 x, 朝向四元数 y, 朝向四元数 z]
std_msgs/Float64MultiArray joint_pos     # 关节角度，[joint1,joint2,...,joint7]

std_msgs/Float64MultiArray init_ee_pose    # 初始末端执行器 (End-Effector) 的位姿信息
std_msgs/Float64MultiArray init_joint_pos    # 初始关节角度

std_msgs/Float64 gripper_width           # 夹爪宽度
std_msgs/UInt8MultiArray rgb_data        # RGB 图像数据， (height x width x 3)

std_msgs/Bool            reset           # 重置信号：True 表示请求系统重置或回到初始状态