# .so
# add_library(libName SHARED source.cpp)
# .a
# add_library(libName STATIC source.cpp)

# static library is recommended to reduce unknown relocation error
# during the compile process.

add_library(myslam STATIC
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp)

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
