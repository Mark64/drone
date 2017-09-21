set(SOURCES dynamic_set.c string_additions.c)
get_filename_component(LIBNAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
add_library(${LIBNAME} STATIC ${SOURCES})
