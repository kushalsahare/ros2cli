cmake_minimum_required(VERSION 3.5)

project(test_ros2cli)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ros_testing REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

  macro(custom_test_process_output test_name config_file_name timeout delay)
    cmake_parse_arguments(_custom_test_process_output
      ""
      "FIXTURE_DELAY;TIMEOUT"
      ${ARGN})

    # Set variables for configuring the test files.
    set(TEST_NAME ${test_name})  # test name used in the test cases
    set(TEST_CONFIGURATION_MODULE ${test_name}_config)  # config file to use
    # Delay between the actions are launch and the ros2 command is run
    if (NOT DEFINED )
      set(_custom_test_process_output_FIXTURE_DELAY 0)
    endif()
    set(FIXTURE_DELAY ${_custom_test_process_output_FIXTURE_DELAY})

    configure_file(
      test/test_process_output_customizable.py.in
      ${test_name}${target_suffix}.py.genexp
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.genexp"
    )
    install(
      FILES test/${config_file_name}.py
      DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
    )
    add_ros_test(
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      TARGET ${test_name}${target_suffix}
      ENV RMW_IMPLEMENTATION=${rmw_implementation}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT ${_custom_test_process_output_TIMEOUT}
    )
    list(
      APPEND generated_python_files
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
    )
  endmacro()

  set(generated_python_files)
  macro(tests)
    set(FIXTURE_DELAY 0.0)
    if("${rmw_implementation}" STREQUAL "rmw_connext_cpp")
      # Connext startup is too slow. It needs a few seconds until discovery starts working.
      set(FIXTURE_DELAY 5.0)
    endif()
    custom_test_process_output(test_ros2action
      FIXTURE_DELAY ${FIXTURE_DELAY} TIMEOUT 180)
    custom_test_process_output(test_ros2topic
      FIXTURE_DELAY ${FIXTURE_DELAY} TIMEOUT 180)
    custom_test_process_output(test_ros2service
      FIXTURE_DELAY ${FIXTURE_DELAY} TIMEOUT 180)
    custom_test_process_output(test_ros2msg TIMEOUT 60)
    custom_test_process_output(test_ros2srv TIMEOUT 60)
  endmacro()
  install(
    FILES test/test_config.py
    DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
  )

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  call_for_each_rmw_implementation(tests)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    TESTNAME "flake8_generated_launch"
    # the generated code might contain longer lines for templated types
    MAX_LINE_LENGTH 999
    ${generated_python_files}
  )
endif()

ament_package()
