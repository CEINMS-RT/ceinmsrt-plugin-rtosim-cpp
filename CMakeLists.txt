PROJECT(RTOSIMplugin CXX)

set(CMAKE_MODULE_PATH
		${CMAKE_SOURCE_DIR}/cmake)

find_package(Concurrency REQUIRED)
find_package(Filter REQUIRED)
find_package(RTOSIM REQUIRED)
find_package(ViconSDK QUIET)
find_package(qualisys_cpp_sdk QUIET)
FIND_PACKAGE(XSD REQUIRED)
FIND_PACKAGE(XercesC REQUIRED)

if(${VICONSDK_FOUND})
	message(STATUS "ViconSDK found")
	add_definitions(-DINCLUDEVICONSDK)
else()
	message(STATUS "ViconSDK not found")
endif()

if(${qualisys_cpp_sdk_FOUND})
	message(STATUS "qualisys_cpp_sdk found")
	add_definitions(-DINCLUDEQUALISYSSDK)
else()
	message(STATUS "qualisys_cpp_sdk not found")
endif()

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

add_subdirectory(src)
