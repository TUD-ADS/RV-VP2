project(peripheral-in-the-loop)

file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h*)
set(SOURCES
	protocol.cpp
	network_io.cpp
	initiator.cpp
	responder.cpp
)

add_library(
	virtual-bus
	${SOURCES}
	${HEADERS}
)

target_include_directories(virtual-bus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(initiator-cli
	initiator-cli.cpp
)
add_executable(responder-cli
	responder-cli.cpp
)
target_compile_features(responder-cli PRIVATE cxx_std_20)
target_link_libraries(initiator-cli virtual-bus pthread)
target_link_libraries(responder-cli virtual-bus pthread)
