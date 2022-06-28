add_executable(vis src/visualize.cpp ${GROUND_REMOVAL_SRC})
target_link_libraries(vis ${PCL_LIBRARIES})
