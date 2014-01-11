INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES __init__.py pouring_task_one_arm.py hrp4_scenario.py tools.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/robohow
)

