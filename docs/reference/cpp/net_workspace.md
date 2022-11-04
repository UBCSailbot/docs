# NET Workspace Structure

## root

The Network Systems directory is similar to the following (READMEs and CMakeLists excluded):

```
network_systems
|   package.xml
└───lib
|   └───protofiles
|       |   message.proto
|       |   ...
|
└───projects
    └───example
    |   └───inc
    |   |   |   example.h
    |   |   |   ...
    |   |
    |   └───src
    |   |   |   example.cpp
    |   |   |   example_subscriber.cpp
    |   |   |   ...
    |   |
    |   └───test
    |   |   |   test_example.cpp
    |   |   |   ...
    |
    └───...

```

At the root of the directory is a `package.xml`. This file tells ROS2 that the network_systems package exists. It is
what allows us to run, for example: `ros2 run network_systems example`.

## lib

The lib is where we will place static libraries that we want to be accessible to all programs. This means they do not
generate their own executable and will always link to a program in the projects directory.

To add new libraries, create a folder and add it to `lib/CMakeLists.txt`. Add a `CMakeLists.txt` file to your newly
created folder and fill it out accordingly.

## projects

Each directory found under projects is module directory. For example, the CAN transceiver will have its own folder in
this directory. Each module will define its executable, unit test executable, and (optionally) its public interface.

Additionally, we will separate the functional source file (`example.cpp`) from the the ROS communication interface file
(`example_subscriber.cpp`). The point is to make the unit tests cover only the functional code, while the communication
code is tackled by integration testing.

To add a new module, create a folder and add it to `projects/CMakeLists.txt`. In your new module folder, add an `inc/`
(optional), `src/`, and `test/` folder, as well as a `CMakeLists.txt` which will need to be filled out accordingly.

??? example
    This is the `CMakeLists.txt` for an example module where the source files are for Cached Fibonacci program.

    ```cmake
    set(module example)

    # Create module library
    set(srcs
        ${CMAKE_CURRENT_LIST_DIR}/src/cached_fib.cpp
    )
    # Make the header accessible to other modules
    add_library(${module} ${srcs})
    target_include_directories(${module} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc ${PROTOBUF_INCLUDE_PATH})

    # Create module ROS executable
    set(bin_module bin_${module})
    set(bin_srcs
        ${srcs}
        ${CMAKE_CURRENT_LIST_DIR}/src/cached_fib_subscriber.cpp
    )
    add_executable(${bin_module} ${bin_srcs})
    ament_target_dependencies(${bin_module} rclcpp std_msgs)
    target_include_directories(${bin_module} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc ${PROTOBUF_INCLUDE_PATH})
    install(TARGETS ${bin_module} DESTINATION lib/${PROJECT_NAME})
    # Rename the output binary to just be the module name
    set_target_properties(${bin_module} PROPERTIES OUTPUT_NAME ${module})

    # Create unit test
    set(test_module test_${module})
    set(test_srcs
        ${srcs}
        ${CMAKE_CURRENT_LIST_DIR}/test/test_cached_fib.cpp
    )
    add_executable(${test_module} ${test_srcs})
    target_include_directories(${test_module} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/inc ${PROTOBUF_INCLUDE_PATH})
    target_link_libraries(${test_module} ${GTEST_LINK_LIBS})
    # Make the unit test runnable with CTest (invoked via test.sh)
    add_test(NAME ${test_module} COMMAND ${test_module})

    ```
