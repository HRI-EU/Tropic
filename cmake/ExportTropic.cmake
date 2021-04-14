

################################################################################
#
# Generate package config files so that the build tree can be referenced by
# other projects.
#
################################################################################

# Settings
SET(PACKAGE_NAME Tropic)
IF(WIN32)
  SET(EXPORT_INSTALL_DEST ${PACKAGE_NAME}/CMake)
ELSE() # Unix etc
  SET(EXPORT_INSTALL_DEST share/cmake/${PACKAGE_NAME})
ENDIF()

# Write version file
INCLUDE(CMakePackageConfigHelpers)
WRITE_BASIC_PACKAGE_VERSION_FILE(
  "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake"
  VERSION 2.0
  COMPATIBILITY AnyNewerVersion
)

# Write targets file
EXPORT(TARGETS ${TROPIC_EXPORT_LIBRARIES}
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Targets.cmake"
#  NAMESPACE Rcs:: # Not really needed, since the targets are prefixed with Rcs anyways
)
# TODO Maybe write to separate target files and use as components?

# Write config file
set(CONFIG_INSTALL_DIR "config")
CONFIGURE_PACKAGE_CONFIG_FILE(
  "cmake/TropicConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake"
  # Hack to ensure the package root is set to the current directory. 
  # Using a custom INSTALL_PREFIX would be better, but that option doesn't
  # exist in CMake 2.8
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}
  PATH_VARS CONFIG_INSTALL_DIR
)

# Add to user package registry if desired
OPTION(WRITE_PACKAGE_REGISTRY "Add build tree to user package registry" OFF)
IF(${WRITE_PACKAGE_REGISTRY})
  EXPORT(PACKAGE ${PACKAGE_NAME})
ENDIF()

################################################################################
#
# Installation of exports
#
################################################################################

# Install exports file
INSTALL(
  EXPORT TropicExport 
  FILE ${PACKAGE_NAME}Targets.cmake
  DESTINATION ${EXPORT_INSTALL_DEST}
)

# Write separate install config to ensure the config dir path is set properly
CONFIGURE_PACKAGE_CONFIG_FILE(
  "cmake/TropicConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/install_files/${PACKAGE_NAME}Config.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/${EXPORT_INSTALL_DEST}"
  PATH_VARS CONFIG_INSTALL_DIR
)
# Install configs and dependency finders
INSTALL(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/install_files/${PACKAGE_NAME}Config.cmake"
#        cmake/Externals.cmake
#        cmake/FindQwt.cmake
  DESTINATION ${EXPORT_INSTALL_DEST}
)
