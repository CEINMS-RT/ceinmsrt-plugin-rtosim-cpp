ADD_LIBRARY(RTOSIMPlugin SHARED 
        ${CMAKE_CURRENT_SOURCE_DIR}/RTOSIMPlugin.cpp
        )

if(${UNFILT})
    SET_TARGET_PROPERTIES(RTOSIMPlugin PROPERTIES COMPILE_FLAGS "-DUNFILT")
endif()

target_include_directories(RTOSIMPlugin PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        )

target_link_libraries(RTOSIMPlugin
        rtb::rtosim
        XMLExecutionInterpreter
		XMLIKIDInterpreter
        osimTools
        XercesC::XercesC
        rtb::Filter
        rtb::Concurrency
        Producers 
		)

if (${QUALISYSSDK_FOUND})

    target_link_libraries(RTOSIMPlugin
            qualisys_cpp_sdk)

ELSEIF (${VICONSDK_FOUND})

    target_link_libraries(RTOSIMPlugin ${ViconSDK_LIBRARIES})

ENDIF ()
