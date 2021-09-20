cmake_minimum_required(VERSION 3.0.2)
project(ltl2ba)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_ltl2ba
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

enable_language(C)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNXT")

add_library(ltl2ba STATIC
			src/parse.c
			src/lex.c
			src/trans.c
			src/buchi.c
			src/set.c
			src/mem.c
			src/rewrt.c
			src/cache.c
			src/alternating.c
			src/generalized.c
			src/ltl2ba.h
)
#target_include_directories(ltl2ba PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_executable(main src/main.c src/ltl2ba.h)
set_target_properties(main PROPERTIES OUTPUT_NAME ltl2ba)
target_link_libraries(main ltl2ba)

