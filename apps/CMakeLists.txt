add_executable(m17-demod m17-demod.cpp)
target_link_libraries(m17-demod PRIVATE m17cxx codec2 Boost::program_options tiny-AES-c)

add_executable(m17-mod m17-mod.cpp)
target_link_libraries(m17-mod PRIVATE m17cxx codec2 Boost::program_options Threads::Threads tiny-AES-c)

add_executable(m17-gateway-link_mod m17-gateway-link_mod.cpp)
target_link_libraries(m17-gateway-link_mod PRIVATE m17cxx codec2 Boost::program_options Threads::Threads external)
IF (WIN32)
	target_link_libraries(m17-gateway-link_mod PRIVATE m17cxx wsock32 ws2_32)
ENDIF()

add_executable(m17-gateway-link_demod m17-gateway-link_demod.cpp)
target_link_libraries(m17-gateway-link_demod PRIVATE m17cxx codec2 Boost::program_options Threads::Threads external)
IF (WIN32)
	target_link_libraries(m17-gateway-link_demod PRIVATE m17cxx wsock32 ws2_32)
ENDIF()

# GUI APPS ONLY FOR WINDOWS
IF (WIN32 AND BUILD_GUI_APPS) 
	add_executable(m17-mod-gui m17-mod-gui.cpp ${IMGUI_SRCS} ${PROJECT_SOURCE_DIR}/src/AudioWrapper.cpp)
	target_link_libraries(m17-mod-gui PRIVATE m17cxx codec2 Boost::program_options Threads::Threads tiny-AES-c ${IMGUI_LIBS} ${PROJECT_SOURCE_DIR}/thirdparty/rtaudio/build/rtaudio.lib external wsock32 ws2_32)
	include_directories(${IMGUI_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/thirdparty/rtaudio)
	install(TARGETS m17-demod m17-mod m17-gateway-link_demod m17-gateway-link_mod m17-mod-gui RUNTIME DESTINATION bin)
ELSEIF(BUILD_GUI_APPS)
	add_executable(m17-mod-gui m17-mod-gui.cpp ${IMGUI_SRCS}  ${PROJECT_SOURCE_DIR}/src/AudioWrapper.cpp)
	target_link_libraries(m17-mod-gui PRIVATE m17cxx codec2 Boost::program_options Threads::Threads tiny-AES-c ${IMGUI_LIBS} ${PROJECT_SOURCE_DIR}/thirdparty/rtaudio/build/librtaudio.so external)
	include_directories(${IMGUI_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/thirdparty/rtaudio)
	install(TARGETS m17-demod m17-mod m17-gateway-link_demod m17-gateway-link_mod m17-mod-gui RUNTIME DESTINATION bin)
ELSE()
	install(TARGETS m17-demod m17-mod m17-gateway-link_demod m17-gateway-link_mod RUNTIME DESTINATION bin)
ENDIF()

