set(IMGUI_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(GLFW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/glfw)
set(BACKENDS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/backends)

list(APPEND IMGUI_INCLUDE_DIR "${BACKENDS_DIR}" "${IMGUI_SRC_DIR}" "${GLFW_DIR}/include")
set(IMGUI_INCLUDE_DIR ${IMGUI_INCLUDE_DIR} CACHE INTERNAL "IMGUI_INCLUDE_DIR")

include_directories(
	${IMGUI_INCLUDE_DIR}
)

file(GLOB_RECURSE IMGUI_SOURCES 
	${IMGUI_SRC_DIR}/*.cpp
	${IMGUI_SRC_DIR}/*.h)

add_library(imgui STATIC ${IMGUI_SOURCES})

find_package(OpenGL REQUIRED)

set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
set(GLFW_INSTALL OFF CACHE INTERNAL "Generate installation target")
add_subdirectory(${GLFW_DIR})

list(APPEND IMGUI_LIBS glfw ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES} imgui)
set(IMGUI_LIBS ${IMGUI_LIBS} CACHE INTERNAL "IMGUI_LIBS")

list(APPEND IMGUI_SRCS ${BACKENDS_DIR}/imgui_impl_glfw.cpp ${BACKENDS_DIR}/imgui_impl_opengl3.cpp)
set(IMGUI_SRCS ${IMGUI_SRCS} CACHE INTERNAL "IMGUI_SRCS")
