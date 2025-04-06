cmake_minimum_required(VERSION 2.8.12)

project(osgAtlasSimbicon)

find_package(DART 6.1.1 REQUIRED COMPONENTS utils-urdf gui-osg)
find_package(PythonLibs 2.7)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
add_definitions(-DWITHOUT_NUMPY=FALSE)

set(NAO_PLUGIN_BASELIB MPCSolver.cpp qpOASES/BLASReplacement.cpp qpOASES/Bounds.cpp qpOASES/Constraints.cpp qpOASES/Flipper.cpp qpOASES/Indexlist.cpp qpOASES/LAPACKReplacement.cpp qpOASES/Matrices.cpp qpOASES/MessageHandling.cpp qpOASES/Options.cpp qpOASES/OQPinterface.cpp qpOASES/QProblem.cpp qpOASES/QProblemB.cpp qpOASES/SolutionAnalysis.cpp qpOASES/SparseSolver.cpp qpOASES/SQProblem.cpp qpOASES/SQProblemSchur.cpp qpOASES/SubjectTo.cpp qpOASES/Utils.cpp
   )

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs} ${NAO_PLUGIN_BASELIB})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})

#set(NAO_PLUGIN_SRC ${NAO_PLUGIN_BASELIB})

#add_library(${PROJECT_NAME} SHARED ${NAO_PLUGIN_SRC})
