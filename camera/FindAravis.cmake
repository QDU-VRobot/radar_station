include(FindPackageHandleStandardArgs)

set(Aravis_VERSION 0.8)
set(Aravis_PATH /usr CACHE PATH "Aravis root path")

# find all include directories
find_path(Aravis_INCLUDE_DIRS arv.h HINTS ${Aravis_PATH} PATH_SUFFIXES "include/aravis-${Aravis_VERSION}")

# find all libraries
find_library(Aravis_LIBS "aravis-${Aravis_VERSION}" HINTS ${Aravis_PATH})

find_package_handle_standard_args(Aravis REQUIRED_VARS Aravis_INCLUDE_DIRS Aravis_LIBS VERSION_VAR Aravis_VERSION)
