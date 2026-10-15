# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target fast_lio::fast_lio
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${fast_lio_TARGETS}.
if(fast_lio_TARGETS AND NOT TARGET fast_lio::fast_lio)
  add_library(fast_lio::fast_lio INTERFACE IMPORTED)
  set_target_properties(fast_lio::fast_lio PROPERTIES
    INTERFACE_LINK_LIBRARIES "${fast_lio_TARGETS}")
endif()
