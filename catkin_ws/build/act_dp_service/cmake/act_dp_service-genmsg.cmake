# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "act_dp_service: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iact_dp_service:/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(act_dp_service_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg" NAME_WE)
add_custom_target(_act_dp_service_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "act_dp_service" "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg" "std_msgs/UInt8MultiArray:std_msgs/Float64MultiArray:std_msgs/Float64:std_msgs/MultiArrayDimension:std_msgs/MultiArrayLayout:std_msgs/Bool"
)

get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv" NAME_WE)
add_custom_target(_act_dp_service_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "act_dp_service" "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv" "std_msgs/UInt8MultiArray:std_msgs/Float64MultiArray:std_msgs/Float64:act_dp_service/RawData:std_msgs/MultiArrayDimension:std_msgs/MultiArrayLayout:std_msgs/Bool"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/act_dp_service
)

### Generating Services
_generate_srv_cpp(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/act_dp_service
)

### Generating Module File
_generate_module_cpp(act_dp_service
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/act_dp_service
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(act_dp_service_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(act_dp_service_generate_messages act_dp_service_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg" NAME_WE)
add_dependencies(act_dp_service_generate_messages_cpp _act_dp_service_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv" NAME_WE)
add_dependencies(act_dp_service_generate_messages_cpp _act_dp_service_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(act_dp_service_gencpp)
add_dependencies(act_dp_service_gencpp act_dp_service_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS act_dp_service_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/act_dp_service
)

### Generating Services
_generate_srv_eus(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/act_dp_service
)

### Generating Module File
_generate_module_eus(act_dp_service
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/act_dp_service
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(act_dp_service_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(act_dp_service_generate_messages act_dp_service_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg" NAME_WE)
add_dependencies(act_dp_service_generate_messages_eus _act_dp_service_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv" NAME_WE)
add_dependencies(act_dp_service_generate_messages_eus _act_dp_service_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(act_dp_service_geneus)
add_dependencies(act_dp_service_geneus act_dp_service_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS act_dp_service_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/act_dp_service
)

### Generating Services
_generate_srv_lisp(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/act_dp_service
)

### Generating Module File
_generate_module_lisp(act_dp_service
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/act_dp_service
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(act_dp_service_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(act_dp_service_generate_messages act_dp_service_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg" NAME_WE)
add_dependencies(act_dp_service_generate_messages_lisp _act_dp_service_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv" NAME_WE)
add_dependencies(act_dp_service_generate_messages_lisp _act_dp_service_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(act_dp_service_genlisp)
add_dependencies(act_dp_service_genlisp act_dp_service_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS act_dp_service_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/act_dp_service
)

### Generating Services
_generate_srv_nodejs(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/act_dp_service
)

### Generating Module File
_generate_module_nodejs(act_dp_service
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/act_dp_service
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(act_dp_service_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(act_dp_service_generate_messages act_dp_service_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg" NAME_WE)
add_dependencies(act_dp_service_generate_messages_nodejs _act_dp_service_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv" NAME_WE)
add_dependencies(act_dp_service_generate_messages_nodejs _act_dp_service_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(act_dp_service_gennodejs)
add_dependencies(act_dp_service_gennodejs act_dp_service_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS act_dp_service_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/act_dp_service
)

### Generating Services
_generate_srv_py(act_dp_service
  "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/act_dp_service
)

### Generating Module File
_generate_module_py(act_dp_service
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/act_dp_service
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(act_dp_service_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(act_dp_service_generate_messages act_dp_service_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/msg/RawData.msg" NAME_WE)
add_dependencies(act_dp_service_generate_messages_py _act_dp_service_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pinkman/projects/dp_depth/catkin_ws/src/act_dp_service/srv/get_action.srv" NAME_WE)
add_dependencies(act_dp_service_generate_messages_py _act_dp_service_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(act_dp_service_genpy)
add_dependencies(act_dp_service_genpy act_dp_service_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS act_dp_service_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/act_dp_service)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/act_dp_service
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(act_dp_service_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(act_dp_service_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/act_dp_service)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/act_dp_service
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(act_dp_service_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(act_dp_service_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/act_dp_service)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/act_dp_service
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(act_dp_service_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(act_dp_service_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/act_dp_service)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/act_dp_service
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(act_dp_service_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(act_dp_service_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/act_dp_service)
  install(CODE "execute_process(COMMAND \"/home/pinkman/isaacsim/kit/python/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/act_dp_service\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/act_dp_service
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(act_dp_service_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(act_dp_service_generate_messages_py geometry_msgs_generate_messages_py)
endif()
