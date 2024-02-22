project( arrmlib )
cmake_minimum_required(VERSION 2.6)

set( LIB_TYPE STATIC )

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
)
endif (WIN32)

set ( SOURCELIST
Afferent/Millard12EqMuscleWithAfferents.cpp
Afferent/Lin02GolgiTendonOrgan.cpp
Afferent/Mileusnic06Spindle.cpp
NeuralController.cpp
ConstraintRod.cpp
Sim.cpp
)

set ( HEADERLIST
Afferent/Millard12EqMuscleWithAfferents.h
Afferent/Lin02GolgiTendonOrgan.h
Afferent/Mileusnic06Spindle.h
NeuralController.h
ConstraintRod.h
Sim.h
)

add_library( arrmlib ${LIB_TYPE} ${HEADERLIST} ${SOURCELIST})

target_link_libraries( arrmlib ${LIBLIST})

SET_TARGET_PROPERTIES(arrmlib PROPERTIES PREFIX "")

install( TARGETS arrmlib DESTINATION lib)

if (WIN32)
set (OPENSIM_DLLS
	${OPENSIM_LIB_DIR}/../../bin/adolc.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_conic_nlpsol.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_conic_qrqp.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_importer_shell.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_integrator_collocation.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_integrator_cvodes.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_integrator_idas.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_integrator_rk.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_interpolant_bspline.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_interpolant_linear.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_linsol_csparse.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_linsol_csparsecholesky.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_linsol_ldl.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_linsol_lsqr.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_linsol_qr.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_linsol_symbolicqr.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_linsol_tridiag.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_nlpsol_ipopt.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_nlpsol_qrsqp.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_nlpsol_scpgen.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_nlpsol_sqpmethod.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_rootfinder_fast_newton.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_rootfinder_kinsol.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_rootfinder_newton.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_rootfinder_nlpsol.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_sundials_common.dll
	${OPENSIM_LIB_DIR}/../../bin/casadi_xmlfile_tinyxml.dll
	${OPENSIM_LIB_DIR}/../../bin/concrt140.dll
	${OPENSIM_LIB_DIR}/../../bin/flang.dll
	${OPENSIM_LIB_DIR}/../../bin/flangrti.dll
	${OPENSIM_LIB_DIR}/../../bin/glut32.dll
	${OPENSIM_LIB_DIR}/../../bin/libblas.dll
	${OPENSIM_LIB_DIR}/../../bin/libgcc_s_sjlj-1.dll
	${OPENSIM_LIB_DIR}/../../bin/libgfortran-3.dll
	${OPENSIM_LIB_DIR}/../../bin/liblapack.dll
	${OPENSIM_LIB_DIR}/../../bin/libomp.dll
	${OPENSIM_LIB_DIR}/../../bin/libquadmath-0.dll
	${OPENSIM_LIB_DIR}/../../bin/msvcp140.dll
	${OPENSIM_LIB_DIR}/../../bin/msvcp140_1.dll
	${OPENSIM_LIB_DIR}/../../bin/msvcp140_2.dll
	${OPENSIM_LIB_DIR}/../../bin/msvcp140_codecvt_ids.dll
	${OPENSIM_LIB_DIR}/../../bin/openblas.dll
	${OPENSIM_LIB_DIR}/../../bin/osimActuators.dll
	${OPENSIM_LIB_DIR}/../../bin/osimAnalyses.dll
	${OPENSIM_LIB_DIR}/../../bin/osimCommon.dll
	${OPENSIM_LIB_DIR}/../../bin/osimExampleComponents.dll
	${OPENSIM_LIB_DIR}/../../bin/osimLepton.dll
	${OPENSIM_LIB_DIR}/../../bin/osimMoco.dll
	${OPENSIM_LIB_DIR}/../../bin/osimSimulation.dll
	${OPENSIM_LIB_DIR}/../../bin/osimTools.dll
	${OPENSIM_LIB_DIR}/../../bin/simbody-visualizer.exe
	${OPENSIM_LIB_DIR}/../../bin/SimTKcommon.dll
	${OPENSIM_LIB_DIR}/../../bin/SimTKmath.dll
	${OPENSIM_LIB_DIR}/../../bin/SimTKsimbody.dll
	${OPENSIM_LIB_DIR}/../../bin/tropter.dll
	${OPENSIM_LIB_DIR}/../../bin/vcruntime140.dll
	${OPENSIM_LIB_DIR}/../../bin/vcruntime140_1.dll
)

install( FILES ${OPENSIM_DLLS} DESTINATION lib)

endif (WIN32)



