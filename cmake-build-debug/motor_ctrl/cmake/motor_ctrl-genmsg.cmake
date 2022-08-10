# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "motor_ctrl: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imotor_ctrl:/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(motor_ctrl_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg" NAME_WE)
add_custom_target(_motor_ctrl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_ctrl" "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg" ""
)

get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg" NAME_WE)
add_custom_target(_motor_ctrl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_ctrl" "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_ctrl
)
_generate_msg_cpp(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_ctrl
)

### Generating Services

### Generating Module File
_generate_module_cpp(motor_ctrl
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_ctrl
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(motor_ctrl_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(motor_ctrl_generate_messages motor_ctrl_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_cpp _motor_ctrl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_cpp _motor_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_ctrl_gencpp)
add_dependencies(motor_ctrl_gencpp motor_ctrl_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_ctrl_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motor_ctrl
)
_generate_msg_eus(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motor_ctrl
)

### Generating Services

### Generating Module File
_generate_module_eus(motor_ctrl
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motor_ctrl
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(motor_ctrl_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(motor_ctrl_generate_messages motor_ctrl_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_eus _motor_ctrl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_eus _motor_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_ctrl_geneus)
add_dependencies(motor_ctrl_geneus motor_ctrl_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_ctrl_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_ctrl
)
_generate_msg_lisp(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_ctrl
)

### Generating Services

### Generating Module File
_generate_module_lisp(motor_ctrl
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_ctrl
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(motor_ctrl_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(motor_ctrl_generate_messages motor_ctrl_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_lisp _motor_ctrl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_lisp _motor_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_ctrl_genlisp)
add_dependencies(motor_ctrl_genlisp motor_ctrl_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_ctrl_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motor_ctrl
)
_generate_msg_nodejs(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motor_ctrl
)

### Generating Services

### Generating Module File
_generate_module_nodejs(motor_ctrl
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motor_ctrl
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(motor_ctrl_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(motor_ctrl_generate_messages motor_ctrl_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_nodejs _motor_ctrl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_nodejs _motor_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_ctrl_gennodejs)
add_dependencies(motor_ctrl_gennodejs motor_ctrl_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_ctrl_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_ctrl
)
_generate_msg_py(motor_ctrl
  "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_ctrl
)

### Generating Services

### Generating Module File
_generate_module_py(motor_ctrl
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_ctrl
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(motor_ctrl_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(motor_ctrl_generate_messages motor_ctrl_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_data.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_py _motor_ctrl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mzy/Code/work_space/UAV_ws/src/motor_ctrl/msg/motor_ctrl.msg" NAME_WE)
add_dependencies(motor_ctrl_generate_messages_py _motor_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_ctrl_genpy)
add_dependencies(motor_ctrl_genpy motor_ctrl_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_ctrl_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_ctrl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_ctrl
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motor_ctrl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motor_ctrl
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_ctrl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_ctrl
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motor_ctrl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motor_ctrl
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_ctrl)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_ctrl\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_ctrl
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
