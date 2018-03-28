project(decision)

#add_subdirectory(behavior_tree)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto DecisionProtoSrc DecisionProtoHds ${ProtoFiles})

add_executable(${PROJECT_NAME}_node
  ${DecisionProtoSrc}
  ${DecisionProtoHds}
  decision_node.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  common::io
  common::log
  perception::map::costmap
  ${catkin_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_test
  decision_test.cpp
  )

target_link_libraries(${PROJECT_NAME}_test
  PRIVATE
  perception::map::costmap
  #        common::io
  #        common::log
  ${catkin_LIBRARIES}
  )

