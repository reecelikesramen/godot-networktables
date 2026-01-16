cmake_minimum_required(VERSION 3.13) # Bumped to 3.13 for target_link_directories
project(nt4_gdext)

# Force C++20 for modern WPILib headers
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 1. Setup Godot-CPP
add_subdirectory(godot-cpp)

file(GLOB SOURCES "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# 2. Include Paths
target_include_directories(${PROJECT_NAME} PRIVATE
    godot-cpp/include
    godot-cpp/gen/include
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# 3. Platform Specific Linking
if(WIN32)
    # Windows x64
    target_link_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/bin/windows/x86_64")
    target_link_libraries(${PROJECT_NAME} PRIVATE godot::cpp ntcore wpinet wpiutil)
    
elseif(APPLE)
    # MacOS Universal
    # We use target_link_directories so order doesn't matter
    target_link_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/bin/macos/universal")
    
    target_link_libraries(${PROJECT_NAME} PRIVATE godot::cpp ntcore wpinet wpiutil)
    
    # RPATH: Look for libs in the same folder as the extension
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "@loader_path")

elseif(UNIX)
    # Linux
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
        target_link_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/bin/linux/arm64")
    else()
        target_link_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/bin/linux/x86_64")
    endif()
    
    target_link_libraries(${PROJECT_NAME} PRIVATE godot::cpp ntcore wpinet wpiutil)
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()