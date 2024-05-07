cmake_minimum_required(VERSION 3.0.2)
project(grid_path_searcher)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    dynamic_reconfigure
    
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

#加载动态配置文件
generate_dynamic_reconfigure_options(
  cfg/Astar_searcher.cfg
)


catkin_package(
  
  INCLUDE_DIRS include
  
)


include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable( demo_node 
    src/demo_node.cpp
    src/Astar_searcher.cpp
    src/read_only/JPS_utils.cpp
    src/read_only/JPS_searcher.cpp
    )

target_link_libraries(demo_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

# 确保在编译demo_node之前生成动态配置文件
add_dependencies(demo_node ${PROJECT_NAME}_gencfg)



add_executable ( random_complex 
    src/random_complex_generator.cpp )

target_link_libraries( random_complex
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} )  
