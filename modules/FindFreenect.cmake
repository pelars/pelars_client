IF(NOT FREENECT_ROOT) 
 IF(EXISTS "/usr/include/libfreenect")
	  SET(FREENECT_ROOT "/usr")
  ELSEIF(EXISTS "/usr/local/include/libfreenect")
	  SET(FREENECT_ROOT "/usr/local")
  ELSE()
    MESSAGE("FREENECT_ROOT not set. Continuing anyway..")
  ENDIF()
ENDIF()

# Include dir
find_path(Freenect_INCLUDE_DIR
  NAMES libfreenect.h
  PATHS ${FREENECT_ROOT}/include/libfreenect 
)

include_directories(${Freenect_INCLUDE_DIR} )

# Finally the library itself
find_library(Freenect_LIBRARY
  NAMES freenect
  PATHS ${FREENECT_ROOT}/lib  /usr/local/lib64/
)