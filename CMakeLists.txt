cmake_minimum_required(VERSION 3.1)

set(PROJECT_VERSION 1.0.0)
set(PROJECT_NAME mc_whycon_plugin)
set(TARGETS_EXPORT_NAME "whycon_Targets")
set(PLUGIN_NAME WhyconPlugin)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

find_package(mc_rtc REQUIRED)
find_package(whycon_plugin_3rd_party_ROS REQUIRED COMPONENTS roscpp whycon_lshape)

set(plugin_SRC
src/LShape.cpp
src/WhyConSubscriber.cpp
src/WhyconPlugin.cpp
#src/WhyConUpdater.cpp
)
set(plugin_HDR
src/LShape.h
src/VisionSubscriber.h
src/WhyConSubscriber.h
src/WhyconPlugin.h
#src/TaskUpdater.h
#src/WhyConUpdater.h
)

add_plugin(${PLUGIN_NAME} "${plugin_SRC}" "${plugin_HDR}")
set_target_properties(${PLUGIN_NAME} PROPERTIES COMPILE_FLAGS "-DMC_WHYCON_PLUGIN_EXPORTS")
target_link_libraries(${PLUGIN_NAME} PUBLIC whycon_plugin::ROS mc_rtc::mc_rtc_ros)
install(FILES etc/${PLUGIN_NAME}.yaml DESTINATION "${MC_PLUGINS_INSTALL_PREFIX}/etc")
