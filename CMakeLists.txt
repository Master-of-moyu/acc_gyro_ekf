# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_executable(main_ekf)
target_sources(main_ekf
    PRIVATE
        main_ekf.cpp
)
target_link_libraries(main_ekf
    PRIVATE
        ${Pangolin_LIBRARIES}
)
