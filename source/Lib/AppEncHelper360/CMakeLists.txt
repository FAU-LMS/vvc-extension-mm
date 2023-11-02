# library
set( LIB_NAME AppEncHelper360 )

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# NATVIS files for Visual Studio
if( MSVC )
  file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )
endif()

# library
add_library( ${LIB_NAME} STATIC ${SRC_FILES} ${INC_FILES} ${NATVIS_FILES} )
target_compile_definitions( ${LIB_NAME} PUBLIC )
target_compile_definitions( ${LIB_NAME} PUBLIC EXTENSION_360_VIDEO=1 )

if( SET_ENABLE_TRACING )
  if( ENABLE_TRACING )
    target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_TRACING=1 )
  else()
    target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_TRACING=0 )
  endif()
endif()

if( ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" )
  target_compile_definitions( ${LIB_NAME} PUBLIC HHI_SPLIT_PARALLELISM=0 )
  target_compile_definitions( ${LIB_NAME} PUBLIC HHI_WPP_PARALLELISM=0 )
else()
  if( SET_HHI_SPLIT_PARALLELISM )
    if( HHI_SPLIT_PARALLELISM )
      target_compile_definitions( ${LIB_NAME} PUBLIC HHI_SPLIT_PARALLELISM=1 )
    else()
      target_compile_definitions( ${LIB_NAME} PUBLIC HHI_SPLIT_PARALLELISM=0 )
    endif()
  endif()
  if( SET_HHI_WPP_PARALLELISM )
    if( HHI_WPP_PARALLELISM )
      target_compile_definitions( ${LIB_NAME} PUBLIC HHI_WPP_PARALLELISM=1 )
    else()
      target_compile_definitions( ${LIB_NAME} PUBLIC HHI_WPP_PARALLELISM=0 )
    endif()
  endif()
endif()

target_include_directories( ${LIB_NAME} PUBLIC . .. )
target_link_libraries( ${LIB_NAME} CommonLib Lib360 )

# example: place header files in different folders
source_group( "Natvis Files" FILES ${NATVIS_FILES} )

# set the folder where to place the projects
set_target_properties( ${LIB_NAME} PROPERTIES FOLDER lib )
