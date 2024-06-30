cmake_minimum_required(VERSION 3.5)
project(libvirtualbreadboard-protocol VERSION 1.0.0 LANGUAGES CXX)


add_library(virtual-breadboard-common src/gpio-common.cpp)
add_library(virtual-breadboard-client src/gpio-client.cpp src/readwrite.cpp)
add_library(virtual-breadboard-server src/gpio-server.cpp src/readwrite.cpp)

target_include_directories(virtual-breadboard-common
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_include_directories(virtual-breadboard-client
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_include_directories(virtual-breadboard-server
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(virtual-breadboard-client virtual-breadboard-common pthread)
target_link_libraries(virtual-breadboard-server virtual-breadboard-common virtual-breadboard-client) 

target_compile_features(virtual-breadboard-common PRIVATE cxx_std_17)
target_compile_features(virtual-breadboard-client PRIVATE cxx_std_17)
target_compile_features(virtual-breadboard-server PRIVATE cxx_std_20)

add_executable(vbb-cli-client test/cli-client.cpp)
add_executable(vbb-cli-server test/cli-server.cpp)
target_link_libraries(vbb-cli-client virtual-breadboard-client)
target_link_libraries(vbb-cli-server virtual-breadboard-server)

install(TARGETS virtual-breadboard-common virtual-breadboard-client virtual-breadboard-server
    EXPORT virtualbreadboard-export
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)