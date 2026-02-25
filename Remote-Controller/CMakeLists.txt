cmake_minimum_required(VERSION 3.16)

# Point to the shared components folder
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../shared_components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(Remote-Controller)
