#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find ROS and some required ROS packages
#
# If everything if found:
# - ROSCPP_FOUND is true
# - you can link with whycon_plugin::ROS
#

if(NOT TARGET whycon_plugin::ROS)
  include(FindPkgConfig)
  pkg_check_modules(WHYCON_PLUGIN_roscpp QUIET roscpp)
  if(${WHYCON_PLUGIN_roscpp_FOUND})
    set(ROSCPP_FOUND True)
    set(WHYCON_PLUGIN_ROS_DEPENDENCIES ${whycon_plugin_3rd_party_ROS_FIND_COMPONENTS})
    foreach(DEP ${WHYCON_PLUGIN_ROS_DEPENDENCIES})
      pkg_check_modules(WHYCON_PLUGIN_${DEP} REQUIRED ${DEP})
      list(APPEND WHYCON_PLUGIN_ROS_LIBRARIES ${WHYCON_PLUGIN_${DEP}_LIBRARIES})
      list(APPEND WHYCON_PLUGIN_ROS_LIBRARY_DIRS ${WHYCON_PLUGIN_${DEP}_LIBRARY_DIRS})
      list(APPEND WHYCON_PLUGIN_ROS_INCLUDE_DIRS ${WHYCON_PLUGIN_${DEP}_INCLUDE_DIRS})
      foreach(FLAG ${WHYCON_PLUGIN_${DEP}_LDFLAGS})
        if(IS_ABSOLUTE ${FLAG})
          list(APPEND WHYCON_PLUGIN_ROS_FULL_LIBRARIES ${FLAG})
        endif()
      endforeach()
    endforeach()
    list(REMOVE_DUPLICATES WHYCON_PLUGIN_ROS_LIBRARIES)
    list(REMOVE_DUPLICATES WHYCON_PLUGIN_ROS_LIBRARY_DIRS)
    list(REMOVE_DUPLICATES WHYCON_PLUGIN_ROS_INCLUDE_DIRS)
    foreach(LIB ${WHYCON_PLUGIN_ROS_LIBRARIES})
      string(SUBSTRING "${LIB}" 0 1 LIB_STARTS_WITH_COLUMN)
      if(${LIB_STARTS_WITH_COLUMN} STREQUAL ":")
        string(SUBSTRING "${LIB}" 1 -1 LIB)
      endif()
      if(IS_ABSOLUTE ${LIB})
        list(APPEND WHYCON_PLUGIN_ROS_FULL_LIBRARIES ${LIB})
      else()
        find_library(${LIB}_FULL_PATH NAME ${LIB} HINTS ${WHYCON_PLUGIN_ROS_LIBRARY_DIRS})
        list(APPEND WHYCON_PLUGIN_ROS_FULL_LIBRARIES ${${LIB}_FULL_PATH})
      endif()
    endforeach()
    list(REMOVE_DUPLICATES WHYCON_PLUGIN_ROS_FULL_LIBRARIES)
    add_library(whycon_plugin::ROS INTERFACE IMPORTED)
    set_target_properties(whycon_plugin::ROS PROPERTIES
      INTERFACE_LINK_LIBRARIES "${WHYCON_PLUGIN_ROS_FULL_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${WHYCON_PLUGIN_ROS_INCLUDE_DIRS}"
    )
    message("-- Found ROS libraries: ${WHYCON_PLUGIN_ROS_FULL_LIBRARIES}")
    message("-- Found ROS include directories: ${WHYCON_PLUGIN_ROS_INCLUDE_DIRS}")
  else()
    set(ROSCPP_FOUND False)
  endif()
endif()
