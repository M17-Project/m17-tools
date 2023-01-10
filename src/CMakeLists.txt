include_directories("${PROJECT_SOURCE_DIR}/include/external")

add_library(m17cxx INTERFACE)

add_subdirectory(tiny-AES-c)

IF(BUILD_GUI_APPS)
	add_subdirectory(imgui)
ENDIF()

add_library(external Log.cpp UDPSocket.cpp)

target_include_directories(m17cxx INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/external>
    $<INSTALL_INTERFACE:include>
    )

target_include_directories(m17cxx INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/tiny-AES-c>
    $<INSTALL_INTERFACE:include>
    )

target_include_directories(m17cxx INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/m17cxx>
    $<INSTALL_INTERFACE:include>
    )

target_compile_features(m17cxx INTERFACE cxx_std_20)

if(MSVC)
    # specify standards-conformance mode
    target_compile_options(m17cxx INTERFACE /permissive-)
    target_compile_definitions(m17cxx INTERFACE _USE_MATH_DEFINES)
endif()

source_group(
    TREE "${PROJECT_SOURCE_DIR}/include"
    PREFIX "Header Files"
    FILES ${HEADER_LIST})

install(TARGETS m17cxx
    EXPORT m17cxxTargets
    LIBRARY DESTINATION lib COMPONENT Runtime
    ARCHIVE DESTINATION lib COMPONENT Development
    RUNTIME DESTINATION bin COMPONENT Runtime
    PUBLIC_HEADER DESTINATION include COMPONENT Development
    BUNDLE DESTINATION bin COMPONENT Runtime
    INCLUDES DESTINATION include
    )
