# Copyright 2013, Francois Keith, JRL, CNRS/AIST

INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

# Add the library to wrap the controller of HRP4.

SET(CONTROLLER_NAME cylinder_pouring)
ADD_LIBRARY(${CONTROLLER_NAME}
  SHARED
  cylinder_pouring.cpp
)


# Link the dynamic library containing the SoT with its dependencies.
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph-python")
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-core")
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-dyninv")
ROSPACK_USE_DEPENDENCY(${CONTROLLER_NAME} "roscpp")
ROSPACK_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic_graph_bridge")

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${Boost_LIBRARIES})
ENDIF(UNIX AND NOT APPLE)


INSTALL(TARGETS ${CONTROLLER_NAME} DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})

# build python submodule
STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${CONTROLLER_NAME})
DYNAMIC_GRAPH_PYTHON_MODULE("sot/robohow/${PYTHON_LIBRARY_NAME}"
  ${CONTROLLER_NAME}
  ${PYTHON_LIBRARY_NAME}-wrap
  )

# Install empty __init__.py files in intermediate directories.
INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/robohow/__init__.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/robohow/gripper.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/robohow/superviser.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/robohow
  )

