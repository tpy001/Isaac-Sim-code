// Auto-generated. Do not edit!

// (in-package act_dp_service.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RawData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ee_pose = null;
      this.joint_pos = null;
      this.init_ee_pose = null;
      this.init_joint_pos = null;
      this.gripper_width = null;
      this.rgb_data = null;
      this.reset = null;
    }
    else {
      if (initObj.hasOwnProperty('ee_pose')) {
        this.ee_pose = initObj.ee_pose
      }
      else {
        this.ee_pose = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('joint_pos')) {
        this.joint_pos = initObj.joint_pos
      }
      else {
        this.joint_pos = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('init_ee_pose')) {
        this.init_ee_pose = initObj.init_ee_pose
      }
      else {
        this.init_ee_pose = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('init_joint_pos')) {
        this.init_joint_pos = initObj.init_joint_pos
      }
      else {
        this.init_joint_pos = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('gripper_width')) {
        this.gripper_width = initObj.gripper_width
      }
      else {
        this.gripper_width = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('rgb_data')) {
        this.rgb_data = initObj.rgb_data
      }
      else {
        this.rgb_data = new std_msgs.msg.UInt8MultiArray();
      }
      if (initObj.hasOwnProperty('reset')) {
        this.reset = initObj.reset
      }
      else {
        this.reset = new std_msgs.msg.Bool();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RawData
    // Serialize message field [ee_pose]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.ee_pose, buffer, bufferOffset);
    // Serialize message field [joint_pos]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.joint_pos, buffer, bufferOffset);
    // Serialize message field [init_ee_pose]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.init_ee_pose, buffer, bufferOffset);
    // Serialize message field [init_joint_pos]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.init_joint_pos, buffer, bufferOffset);
    // Serialize message field [gripper_width]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.gripper_width, buffer, bufferOffset);
    // Serialize message field [rgb_data]
    bufferOffset = std_msgs.msg.UInt8MultiArray.serialize(obj.rgb_data, buffer, bufferOffset);
    // Serialize message field [reset]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.reset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RawData
    let len;
    let data = new RawData(null);
    // Deserialize message field [ee_pose]
    data.ee_pose = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_pos]
    data.joint_pos = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [init_ee_pose]
    data.init_ee_pose = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [init_joint_pos]
    data.init_joint_pos = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [gripper_width]
    data.gripper_width = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [rgb_data]
    data.rgb_data = std_msgs.msg.UInt8MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [reset]
    data.reset = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.ee_pose);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.joint_pos);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.init_ee_pose);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.init_joint_pos);
    length += std_msgs.msg.UInt8MultiArray.getMessageSize(object.rgb_data);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'act_dp_service/RawData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a2490b5f4d8422f6e431911c53295728';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64MultiArray ee_pose     # 当前末端执行器 (End-Effector) 的位姿信息, 数据格式为一维数组: [位置 x, 位置 y, 位置 z, 朝向四元数 w, 朝向四元数 x, 朝向四元数 y, 朝向四元数 z]
    std_msgs/Float64MultiArray joint_pos     # 关节角度，[joint1,joint2,...,joint7]
    
    std_msgs/Float64MultiArray init_ee_pose    # 初始末端执行器 (End-Effector) 的位姿信息
    std_msgs/Float64MultiArray init_joint_pos    # 初始关节角度
    
    std_msgs/Float64 gripper_width           # 夹爪宽度
    std_msgs/UInt8MultiArray rgb_data        # RGB 图像数据， (height x width x 3)
    
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
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RawData(null);
    if (msg.ee_pose !== undefined) {
      resolved.ee_pose = std_msgs.msg.Float64MultiArray.Resolve(msg.ee_pose)
    }
    else {
      resolved.ee_pose = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.joint_pos !== undefined) {
      resolved.joint_pos = std_msgs.msg.Float64MultiArray.Resolve(msg.joint_pos)
    }
    else {
      resolved.joint_pos = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.init_ee_pose !== undefined) {
      resolved.init_ee_pose = std_msgs.msg.Float64MultiArray.Resolve(msg.init_ee_pose)
    }
    else {
      resolved.init_ee_pose = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.init_joint_pos !== undefined) {
      resolved.init_joint_pos = std_msgs.msg.Float64MultiArray.Resolve(msg.init_joint_pos)
    }
    else {
      resolved.init_joint_pos = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.gripper_width !== undefined) {
      resolved.gripper_width = std_msgs.msg.Float64.Resolve(msg.gripper_width)
    }
    else {
      resolved.gripper_width = new std_msgs.msg.Float64()
    }

    if (msg.rgb_data !== undefined) {
      resolved.rgb_data = std_msgs.msg.UInt8MultiArray.Resolve(msg.rgb_data)
    }
    else {
      resolved.rgb_data = new std_msgs.msg.UInt8MultiArray()
    }

    if (msg.reset !== undefined) {
      resolved.reset = std_msgs.msg.Bool.Resolve(msg.reset)
    }
    else {
      resolved.reset = new std_msgs.msg.Bool()
    }

    return resolved;
    }
};

module.exports = RawData;
