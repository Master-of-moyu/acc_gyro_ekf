# -----------------------------------------------------------
add_executable(coordinate_transformation)
target_sources(coordinate_transformation
    PRIVATE
    coordinate_transformation.cpp
)
target_link_libraries(coordinate_transformation
    PRIVATE
        ${Pangolin_LIBRARIES}
)
# -----------------------------------------------------------
