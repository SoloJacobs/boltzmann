add_library(matrix matrix.cpp)
target_include_directories(matrix PUBLIC .)

# Install instructions
set_target_properties(matrix PROPERTIES PUBLIC_HEADER matrix.hpp PUBLIC_HEADER matrixslice.hpp)
install(TARGETS matrix 
				LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib/
				ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/lib/
				PUBLIC_HEADER DESTINATION ${PROJECT_SOURCE_DIR}/include/
				)
