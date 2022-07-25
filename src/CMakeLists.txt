set(plugin_SRC
LShape.cpp
WhyConSubscriber.cpp
WhyconPlugin.cpp
WhyConUpdater.cpp
)
set(plugin_HDR
../include/mc_whycon_plugin/LShape.h
../include/mc_whycon_plugin/VisionSubscriber.h
../include/mc_whycon_plugin/WhyConSubscriber.h
../include/mc_whycon_plugin/WhyconPlugin.h
../include/mc_whycon_plugin/TaskUpdater.h
../include/mc_whycon_plugin/WhyConUpdater.h
)

option(AUTOLOAD_${PLUGIN_NAME}_PLUGIN "Automatically load ${PLUGIN_NAME} plugin" OFF)
add_plugin(${PLUGIN_NAME} "${plugin_SRC}" "${plugin_HDR}")
target_include_directories(${PLUGIN_NAME} PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)
set_target_properties(${PLUGIN_NAME} PROPERTIES COMPILE_FLAGS "-DMC_WHYCON_PLUGIN_EXPORTS")
target_link_libraries(${PLUGIN_NAME} PUBLIC whycon_plugin::ROS mc_rtc::mc_rtc_ros)

add_subdirectory(states)
