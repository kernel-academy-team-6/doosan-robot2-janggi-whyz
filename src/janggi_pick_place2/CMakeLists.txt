cmake_minimum_required(VERSION 3.8)
project(janggi_pick_place2)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Home.action"
  "action/Pick.action"
  "action/Place.action"
  "action/PickAndPlace.action"
)
# rosidl installs the Python package and action subpackage. Install runtime
# modules alongside it without a second ament_python_install_package target.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY janggi_pick_place2/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/pick_place_node scripts/pick_place_example
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(core tests/test_core.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(gripper_calibration tests/test_gripper_calibration.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(virtual_backend tests/test_virtual.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(runtime_factory tests/test_runtime.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(simulation_feedback tests/test_simulation_feedback.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(joint6_guard tests/test_joint6.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(locked_wrist tests/test_locked_wrist.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(command_only tests/test_command_only.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
