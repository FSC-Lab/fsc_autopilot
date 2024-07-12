if(PROJECT_IS_TOP_LEVEL)
  set(CMAKE_INSTALL_INCLUDEDIR
      "include/${CMAKE_PROJECT_NAME}-${PROJECT_VERSION}"
      CACHE PATH "")
endif()

# Project is configured with no languages, so tell GNUInstallDirs the lib dir
set(CMAKE_INSTALL_LIBDIR
    lib
    CACHE PATH "")

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# find_package(<package>) call for consumers to find this project
set(package ${CMAKE_PROJECT_NAME})

install(
  DIRECTORY ${PROJECT_INCLUDE_DIRS}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT ${CMAKE_PROJECT_NAME}_Development
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

install(
  TARGETS ${PROJECT_TARGETS}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  INCLUDES
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

write_basic_package_version_file(
  "${package}ConfigVersion.cmake" COMPATIBILITY SameMajorVersion
                                                ARCH_INDEPENDENT)

# Allow package maintainers to freely override the path for the configs
set(${CMAKE_PROJECT_NAME}_INSTALL_CMAKEDIR
    "${CMAKE_INSTALL_LIBDIR}/cmake/${package}"
    CACHE PATH "CMake package config location relative to the install prefix")
mark_as_advanced(${CMAKE_PROJECT_NAME}_INSTALL_CMAKEDIR)

configure_package_config_file(
  cmake/install_config.cmake.in "${CMAKE_BINARY_DIR}/${package}Config.cmake"
  INSTALL_DESTINATION "${${CMAKE_PROJECT_NAME}_INSTALL_CMAKEDIR}"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR)

install(
  FILES "${CMAKE_BINARY_DIR}/${package}Config.cmake"
  DESTINATION "${${CMAKE_PROJECT_NAME}_INSTALL_CMAKEDIR}"
  COMPONENT ${CMAKE_PROJECT_NAME}_Development)

install(
  FILES "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake"
  DESTINATION "${${CMAKE_PROJECT_NAME}_INSTALL_CMAKEDIR}"
  COMPONENT ${CMAKE_PROJECT_NAME}_Development)

install(
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  NAMESPACE ${CMAKE_PROJECT_NAME}::
  DESTINATION "${${CMAKE_PROJECT_NAME}_INSTALL_CMAKEDIR}"
  COMPONENT ${CMAKE_PROJECT_NAME}_Development)

if(PROJECT_IS_TOP_LEVEL)
  include(CPack)
endif()
