project( RunARRM )
cmake_minimum_required(VERSION 2.6)

set(OPENSIM_INCLUDE_DIR "" CACHE PATH "OpenSim Include directory")
set(OPENSIM_DEP_BASE_DIR "" CACHE PATH "OpenSim Dependencies Intalled directory")
set(OPENSIM_LIB_DIR "" CACHE PATH "OpenSim library directory")
set(OPENSIM_DEP_LIB_DIR "" CACHE PATH "OpenSim Dependencies SimBody library directory")

if (WIN32)
include_directories( ${OPENSIM_INCLUDE_DIR} )
include_directories( ${OPENSIM_INCLUDE_DIR}/OpenSim )
include_directories( ${OPENSIM_DEP_BASE_DIR}/simbody/include )
include_directories( ${OPENSIM_DEP_BASE_DIR}/spdlog/include )
else (WIN32)
include_directories( ${OPENSIM_INCLUDE_DIR} )
include_directories( ${OPENSIM_INCLUDE_DIR}/OpenSim )
include_directories( ${OPENSIM_DEP_BASE_DIR}/simbody/include/simbody )
include_directories( ${OPENSIM_DEP_BASE_DIR}/spdlog/include/spdlog )
endif (WIN32)
include_directories( ../arrmlib )
link_directories(${OPENSIM_LIB_DIR})
link_directories(${OPENSIM_DEP_LIB_DIR})

if (WIN32)
set ( LIBLIST
	${OPENSIM_LIB_DIR}/osimTools.lib
	${OPENSIM_LIB_DIR}/osimActuators.lib
	${OPENSIM_LIB_DIR}/osimAnalyses.lib
	${OPENSIM_LIB_DIR}/osimCommon.lib
	${OPENSIM_LIB_DIR}/osimExampleComponents.lib
	${OPENSIM_LIB_DIR}/osimLepton.lib
	${OPENSIM_LIB_DIR}/osimSimulation.lib
	${OPENSIM_DEP_LIB_DIR}/SimTKsimbody.lib
	${OPENSIM_DEP_LIB_DIR}/SimTKmath.lib
	${OPENSIM_DEP_LIB_DIR}/SimTKcommon.lib
	arrmlib
)
else (WIN32)
set ( LIBLIST
	libosimTools.so
	libosimActuators.so
	libosimAnalyses.so
	libosimCommon.so
	libosimExampleComponents.so
	libosimLepton.so
	libosimSimulation.so
	libSimTKsimbody.so
	libSimTKmath.so
	libSimTKcommon.so
	arrmlib
)
endif (WIN32)

add_executable(RunARRM arm.cpp)
target_link_libraries(RunARRM  ${LIBLIST})