add_executable(Executable main.cpp)

find_package( OpenCV REQUIRED )
target_include_directories( Executable PUBLIC ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( Executable ${OpenCV_LIBS} )
