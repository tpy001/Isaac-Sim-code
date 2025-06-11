// Auto-generated. Do not edit!

// (in-package act_dp_service.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RawData = require('../msg/RawData.js');

//-----------------------------------------------------------

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class get_actionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sensor = null;
    }
    else {
      if (initObj.hasOwnProperty('sensor')) {
        this.sensor = initObj.sensor
      }
      else {
        this.sensor = new RawData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_actionRequest
    // Serialize message field [sensor]
    bufferOffset = RawData.serialize(obj.sensor, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_actionRequest
    let len;
    let data = new get_actionRequest(null);
    // Deserialize message field [sensor]
    data.sensor = RawData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += RawData.getMessageSize(object.sensor);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'act_dp_service/get_actionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '335520706867ed6fa97cbd3d60f4c7b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 输入: 传感器数据
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
    const resolved = new get_actionRequest(null);
    if (msg.sensor !== undefined) {
      resolved.sensor = RawData.Resolve(msg.sensor)
    }
    else {
      resolved.sensor = new RawData()
    }

    return resolved;
    }
};

class get_actionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.actions = null;
    }
    else {
      if (initObj.hasOwnProperty('actions')) {
        this.actions = initObj.actions
      }
      else {
        this.actions = new std_msgs.msg.Float64MultiArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_actionResponse
    // Serialize message field [actions]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.actions, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_actionResponse
    let len;
    let data = new get_actionResponse(null);
    // Deserialize message field [actions]
    data.actions = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.actions);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'act_dp_service/get_actionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a0770e10a9dd25aa1532dcce6ecc6c4d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new get_actionResponse(null);
    if (msg.actions !== undefined) {
      resolved.actions = std_msgs.msg.Float64MultiArray.Resolve(msg.actions)
    }
    else {
      resolved.actions = new std_msgs.msg.Float64MultiArray()
    }

    return resolved;
    }
};

module.exports = {
  Request: get_actionRequest,
  Response: get_actionResponse,
  md5sum() { return 'bb5933a9fa74805641f297a01d40ec1d'; },
  datatype() { return 'act_dp_service/get_action'; }
};
