add_executable(Executable main.cpp)

find_package( OpenCV REQUIRED )
target_include_directories( Executable PUBLIC ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( Executable ${OpenCV_LIBS} )

add_executable(Test_Codec test_codec.cpp)
target_include_directories( Test_Codec PUBLIC ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( Test_Codec ${OpenCV_LIBS} )
