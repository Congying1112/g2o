INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
project(2dslam_with_scale)

ADD_EXECUTABLE(${PROJECT_NAME}
        2dslam_with_scale.cpp
        types_scaled_2d.cpp
        vertex_se2_with_scale.cpp
        edge_se2_with_scale.cpp)

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME}${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} core solver_csparse)
