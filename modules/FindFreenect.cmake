
find_path(Freenect_INCLUDE_DIR
  NAMES libfreenect.h
  PATHS /prefix/include/libfreenect 
)

include_directories(${Freenect_INCLUDE_DIR} )

# Finally the library itself
find_library(Freenect_LIBRARY
  NAMES freenect
  PATHS /prefix/lib/  /usr/local/lib64/
)