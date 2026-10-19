cmake_minimum_required(VERSION 3.8)
project(taskweave_robots NONE)
find_package(ament_cmake REQUIRED)

# Every folder is installed to share/taskweave_robots/<robot>/, except the tests, caches and hidden folders. The copy
# happens at install time, so a new pack folder needs no CMake reconfigure (no configure-time GLOB).
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION share/${PROJECT_NAME}
        PATTERN "test" EXCLUDE
        PATTERN "__pycache__" EXCLUDE
        PATTERN ".*" EXCLUDE
        PATTERN "CMakeLists.txt" EXCLUDE
        PATTERN "package.xml" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # the packs are checked in the source tree (test/ finds them next to itself) against the other packages' code
  ament_add_pytest_test(test_robot_packs test TIMEOUT 300)
endif()

ament_package()
