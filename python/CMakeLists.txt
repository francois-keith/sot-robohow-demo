INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES __init__.py pouring_task_one_arm.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/robohow
)

