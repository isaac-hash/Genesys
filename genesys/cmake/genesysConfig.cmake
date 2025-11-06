# --- genesysConfig.cmake ---
if(NOT TARGET genesys::genesys)
  # Include the helper file that sets genesys_INCLUDE_DIRS
  include("${CMAKE_CURRENT_LIST_DIR}/genesys.cmake")

  # Create an INTERFACE library target.
  # This is a "virtual" target that doesn't build anything but can have properties.
  add_library(genesys::genesys INTERFACE IMPORTED)

  # Set the include directories for any target that links against genesys::genesys
  set_target_properties(genesys::genesys PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${genesys_INCLUDE_DIRS}"
  )
endif()
