cmake_minimum_required(VERSION 3.9)
project(turtlelib)

# create the turtlelib library
add_library(${PROJECT_NAME} src/rigid2d.cpp src/diff_drive.cpp src/kalman.cpp)
# The add_library function just added turtlelib as a "target"
# A "target" is a name that CMake uses to refer to some type of output
# In this case it is a library but it could also be an executable or some other items

# Public include directories can be used by other targets that link against turtlelib
# By adding include/ to the include path, this means that files in e.g., include/turtlelib
# can be included with #include"turtlelib/file.hpp"
# target_include_directories(${PROJECT_NAME} PUBLIC include/)

find_package(Armadillo)
include_directories(${ARMADILLO_INCLUDE_DIRS})

# enable C++ 17
target_compile_features(turtlelib PUBLIC cxx_std_17)

# warnings are your friend!
target_compile_options(turtlelib PUBLIC -Wall -Wextra -Wpedantic)

# create the executable target  and link it with the rigid2d library
# It is also possible specify multiple cpp files and they will be linked
# into a single executable (as long as exactly one of these files includes a main() function).
# However, by creating a library (as we are doing here) the library files
# can be compiled once and used
add_executable(frame_main src/frame_main.cpp)

target_link_libraries(frame_main ${PROJECT_NAME})

target_include_directories(turtlelib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/>)

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Install the targets and create a CMake Exported Target
# The CMake Exported Target can be used to access this project
# from other CMake projects, after installation
# The targets will be installed to default locations
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets)

# The project_name-targets now also needs to be exported.
# This call will generate a file called project_name-config.cmake
# That contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use your library
install(EXPORT ${PROJECT_NAME}-targets
        FILE ${PROJECT_NAME}-config.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/${PROJECT_NAME})


# CMake also has the ability to generate doxygen documentation


if(NOT CMAKE_CROSSCOMPILING)
find_package(Doxygen)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
doxygen_add_docs(doxygen include/ src/ README.md ALL)
endif()

# Use the cmake testing functionality. A test is just an executable.
# We use the catch2 unit testing framework
find_package(Catch2 3 REQUIRED)

# enable_testing()
include(CTest)
add_executable(turtlelib_test tests/tests.cpp)
target_link_libraries(turtlelib_test turtlelib Catch2::Catch2WithMain)

add_test(NAME Test_of_Turtlelib COMMAND turtlelib_test)
