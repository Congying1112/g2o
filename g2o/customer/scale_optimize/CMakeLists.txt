INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
project(scale_optimize)

ADD_EXECUTABLE(${PROJECT_NAME}
        scale_optimize.cpp
        types_scale.cpp
        VertexScale.cpp
        EdgeScale.cpp
        )

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME}${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} core solver_csparse)
