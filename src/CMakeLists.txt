add_executable(blind_controller_node
  blind_controller_node.cpp
  #  blind_controller.cpp
  )

add_dependencies(blind_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(blind_controller_node ${catkin_LIBRARIES})
