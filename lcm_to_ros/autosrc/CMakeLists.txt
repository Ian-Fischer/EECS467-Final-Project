add_executable(example_topic_republisher example_topic_republisher.cpp)
target_link_libraries( example_topic_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(example_topic_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(other_topic_republisher other_topic_republisher.cpp)
target_link_libraries( other_topic_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(other_topic_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(ODOMETRY_republisher ODOMETRY_republisher.cpp)
target_link_libraries( ODOMETRY_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(ODOMETRY_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(MBOT_MOTOR_COMMAND_republisher MBOT_MOTOR_COMMAND_republisher.cpp)
target_link_libraries( MBOT_MOTOR_COMMAND_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(MBOT_MOTOR_COMMAND_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


