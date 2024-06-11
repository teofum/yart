# Library definition
add_library(metal_cpp ${CMAKE_CURRENT_SOURCE_DIR}/impl.cpp)

# Metal cpp headers
target_include_directories(metal_cpp PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/metal-cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/metal-cpp-extensions"
)

# Metal cpp library (linker)
target_link_libraries(metal_cpp
        "-framework Metal"
        "-framework MetalKit"
        "-framework AppKit"
        "-framework Foundation"
        "-framework QuartzCore"
)