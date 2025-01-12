add_executable(lvio_fusion_node
        lvio_fusion_node.cpp
        parameters.cpp
        visualization.cpp
        camera_pose.cpp
)

target_link_libraries(lvio_fusion_node ${FMT_LIBRARIES} ${THIRD_PARTY_LIBS})

add_executable(path2tum_node path2tum.cpp)
target_link_libraries(path2tum_node ${catkin_LIBRARIES})

target_compile_features(lvio_fusion_node PRIVATE cxx_std_14)