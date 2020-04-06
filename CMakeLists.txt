cmake_minimum_required(VERSION 3.1)

set(PROJECT_VERSION 1.0.0)
set(PROJECT_NAME mc_whycon_plugin)
set(TARGETS_EXPORT_NAME "whycon_Targets")
set(PLUGIN_NAME WhyconPlugin)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

find_package(mc_rtc REQUIRED)
find_package(whycon_plugin_3rd_party_ROS REQUIRED COMPONENTS roscpp whycon_lshape)

add_subdirectory(src)
install(FILES etc/${PLUGIN_NAME}.yaml DESTINATION "${MC_PLUGINS_INSTALL_PREFIX}/etc")
