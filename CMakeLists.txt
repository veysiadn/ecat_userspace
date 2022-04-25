cmake_minimum_required(VERSION 3.5)
project(ecat_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
## Compile options for IgH libary and several Linux libraries (e.g lpthread)
add_compile_options(-g -w -Wall -Wextra -Wpedantic -I/opt/etherlab/include -L/opt/etherlab/lib -lethercat -lpthread -lrt)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

## Defining paths and libraries to include in the next section.
set(etherlab_include /opt/etherlab/include)
set(etherlab_lib /opt/etherlab/lib/libethercat.so.1.1.0)
set(ecat_node_include ${CMAKE_CURRENT_SOURCE_DIR}/include/)
set(node_name "ecat_node")

## Finding packages that'll be required for compilation.
## Don't forget to add packages if you use it in your code, otherwise you'll get build errors.

## Output executable name and requied cpp files for executable
file(GLOB ecat_node_SRC
     "src/*.cpp"
)

add_executable(ecat_node ${ecat_node_SRC})

## Specifying include directories for ecat_node specifically by using definitions above.
## target include directories adds include directory for specific target executable.
## you can add directory in this way or you can use 
# include_directories(x/y) for general include for whole project..
target_include_directories(ecat_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${etherlab_include})

## Specifying libraries by using definitions above.
target_link_libraries(ecat_node
PRIVATE Threads::Threads
${etherlab_lib}
)
# Add include directories
include_directories(
  ${etherlab_include}
  ${ecat_node_include}
)

## Don't forget to add dependencies to your build file, 
## Use find_package(x) then add dependecy for x. 

install(TARGETS ecat_node
  DESTINATION build/${PROJECT_NAME})