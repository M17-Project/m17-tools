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

install(TARGETS m17-demod m17-mod m17-gateway-link_demod m17-gateway-link_mod RUNTIME DESTINATION bin)
