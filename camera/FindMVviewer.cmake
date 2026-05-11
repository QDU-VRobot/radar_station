include(FindPackageHandleStandardArgs)

set(MVVIEWER_SDK_PATH /opt/HuarayTech/MVviewer CACHE PATH "MVviewer root path")

# find all include directories
find_path(MVviewer_INCLUDE_DIRS IMVApi.h HINTS ${MVVIEWER_SDK_PATH} PATH_SUFFIXES include)

# find all libraries
find_library(MVviewer_LIBS MVSDK HINTS ${MVVIEWER_SDK_PATH} PATH_SUFFIXES lib)

# find all source files
#aux_source_directory()
#set(MVviewer_SOURCE )

find_package_handle_standard_args(MVviewer DEFAULT_MSG MVviewer_INCLUDE_DIRS MVviewer_LIBS #[[MVviewer_SOURCE]])
