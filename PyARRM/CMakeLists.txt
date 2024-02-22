project( _arrm )
cmake_minimum_required(VERSION 2.6)

set( LIB_TYPE SHARED )

set( Python3_USE_STATIC_LIBS FALSE )

if (APPLE)
if (SKBUILD) # We're building using SKBUILD which means the correct python lib is passed as a flag! Hooray!
set(Python3_INCLUDE_DIRS ${PYTHON_INCLUDE_DIR} )
set(PYTHON_LIBRARIES ${PYTHON_LIBRARY})
set(Python3_LIBRARIES ${PYTHON_LIBRARY})
include_directories(${Python3_INCLUDE_DIRS})
else (SKBUILD)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
endif(SKBUILD)
else (APPLE)
if (WIN32 AND SKBUILD) # We're building using SKBUILD which means the correct python lib is passed as a flag! Hooray!
set(Python3_INCLUDE_DIRS ${PYTHON_INCLUDE_DIR} )
set(PYTHON_LIBRARIES ${PYTHON_LIBRARY})
set(Python3_LIBRARIES ${PYTHON_LIBRARY})
include_directories(${Python3_INCLUDE_DIRS})
else (WIN32 AND SKBUILD)
find_package (Python3 COMPONENTS Interpreter Development)
include_directories(${Python3_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIR})
endif (WIN32 AND SKBUILD)
endif(APPLE)

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
	osimTools
	osimActuators
	osimAnalyses
	osimCommon
	osimExampleComponents
	osimLepton
	osimSimulation
	SimTKsimbody
	SimTKmath
	SimTKcommon
	${PYTHON_LIBRARIES}
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
	${PYTHON_LIBRARIES}
	arrmlib
)
endif (WIN32)

set ( SOURCELIST
sim_wrap.cxx
)

set ( HEADERLIST
)

add_library( _arrm ${LIB_TYPE} ${HEADERLIST} ${SOURCELIST})

target_link_libraries( _arrm ${LIBLIST})

SET_TARGET_PROPERTIES(_arrm PROPERTIES PREFIX "")

if (WIN32) # for windows, we want to set the ending to .pyd. In linux, the ending is just .so
SET_TARGET_PROPERTIES(_arrm   PROPERTIES SUFFIX .pyd)
endif (WIN32)

if (APPLE) # for osx, we want to force the ending to .so instead of .dylib.
SET_TARGET_PROPERTIES(_arrm   PROPERTIES SUFFIX .so)
endif (APPLE)

install( TARGETS _arrm DESTINATION lib)
install( FILES arrm.py DESTINATION lib)
install( FILES ../MoBL_ARMS_module2_4_onemuscle_afferent.osim DESTINATION lib)
install( FILES ../MoBL_ARMS_module2_4_onemuscle_afferent_no_string.osim DESTINATION lib)

set ( GeometryList
	../Geometry/1mc.vtp
	../Geometry/2distph.vtp
	../Geometry/2mc.vtp
	../Geometry/2midph.vtp
	../Geometry/2proxph.vtp
	../Geometry/3distph.vtp
	../Geometry/3mc.vtp
	../Geometry/3midph.vtp
	../Geometry/3proxph.vtp
	../Geometry/4distph.vtp
	../Geometry/4mc.vtp
	../Geometry/4midph.vtp
	../Geometry/4proxph.vtp
	../Geometry/5distph.vtp
	../Geometry/5mc.vtp
	../Geometry/5midph.vtp
	../Geometry/5proxph.vtp
	../Geometry/arm_r_1mc.vtp
	../Geometry/arm_r_2distph.vtp
	../Geometry/arm_r_2mc.vtp
	../Geometry/arm_r_2midph.vtp
	../Geometry/arm_r_2proxph.vtp
	../Geometry/arm_r_3distph.vtp
	../Geometry/arm_r_3mc.vtp
	../Geometry/arm_r_3midph.vtp
	../Geometry/arm_r_3proxph.vtp
	../Geometry/arm_r_4distph.vtp
	../Geometry/arm_r_4mc.vtp
	../Geometry/arm_r_4midph.vtp
	../Geometry/arm_r_4proxph.vtp
	../Geometry/arm_r_5distph.vtp
	../Geometry/arm_r_5mc.vtp
	../Geometry/arm_r_5midph.vtp
	../Geometry/arm_r_5proxph.vtp
	../Geometry/arm_r_capitate.vtp
	../Geometry/arm_r_hamate.vtp
	../Geometry/arm_r_humerus.vtp
	../Geometry/arm_r_lunate.vtp
	../Geometry/arm_r_pisiform.vtp
	../Geometry/arm_r_radius.vtp
	../Geometry/arm_r_scaphoid.vtp
	../Geometry/arm_r_thumbdist.vtp
	../Geometry/arm_r_thumbprox.vtp
	../Geometry/arm_r_trapezium.vtp
	../Geometry/arm_r_trapezoid.vtp
	../Geometry/arm_r_triquetrum.vtp
	../Geometry/arm_r_ulna.vtp
	../Geometry/arrow.vtp
	../Geometry/axes.vtp
	../Geometry/bofoot.vtp
	../Geometry/box.vtp
	../Geometry/bucket.vtp
	../Geometry/calcaneous_lv.vtp
	../Geometry/calcaneous_rv.vtp
	../Geometry/capitate.vtp
	../Geometry/capitate_lvs.vtp
	../Geometry/capitate_rvs.vtp
	../Geometry/cerv1.vtp
	../Geometry/cerv1sm.vtp
	../Geometry/cerv2.vtp
	../Geometry/cerv2sm.vtp
	../Geometry/cerv3.vtp
	../Geometry/cerv3sm.vtp
	../Geometry/cerv4.vtp
	../Geometry/cerv4sm.vtp
	../Geometry/cerv5.vtp
	../Geometry/cerv5sm.vtp
	../Geometry/cerv6.vtp
	../Geometry/cerv6sm.vtp
	../Geometry/cerv7.vtp
	../Geometry/clavicle.vtp
	../Geometry/clavicle_lv.vtp
	../Geometry/clavicle_lvsm.vtp
	../Geometry/clavicle_rv.vtp
	../Geometry/clavicle_rvsm.vtp
	../Geometry/cone.vtp
	../Geometry/Cube.vtp
	../Geometry/cuboid_lv.vtp
	../Geometry/cuboid_rv.vtp
	../Geometry/cylinder.vtp
	../Geometry/distal_phalanx1_lvs.vtp
	../Geometry/distal_phalanx1_rvs.vtp
	../Geometry/distal_phalanx2_lvs.vtp
	../Geometry/distal_phalanx2_rvs.vtp
	../Geometry/distal_phalanx3_lvs.vtp
	../Geometry/distal_phalanx3_rvs.vtp
	../Geometry/distal_phalanx4_rvs.vtp
	../Geometry/distal_phalanx5_lvs.vtp
	../Geometry/distal_phalanx5_rvs.vtp
	../Geometry/femur.vtp
	../Geometry/femur_l.vtp
	../Geometry/femur_r.vtp
	../Geometry/fibula.vtp
	../Geometry/fibula_l.vtp
	../Geometry/fibula_r.vtp
	../Geometry/fingers1.vtp
	../Geometry/fingers10.vtp
	../Geometry/fingers11.vtp
	../Geometry/fingers12.vtp
	../Geometry/fingers13.vtp
	../Geometry/fingers14.vtp
	../Geometry/fingers15.vtp
	../Geometry/fingers16.vtp
	../Geometry/fingers17.vtp
	../Geometry/fingers18.vtp
	../Geometry/fingers19.vtp
	../Geometry/fingers2.vtp
	../Geometry/fingers3.vtp
	../Geometry/fingers4.vtp
	../Geometry/fingers5.vtp
	../Geometry/fingers6.vtp
	../Geometry/fingers7.vtp
	../Geometry/fingers8.vtp
	../Geometry/fingers9.vtp
	../Geometry/first_cuneiform_lv.vtp
	../Geometry/first_cuneiform_rv.vtp
	../Geometry/foot.vtp
	../Geometry/ground_jaw.vtp
	../Geometry/ground_r_clavicle.vtp
	../Geometry/ground_r_scapula.vtp
	../Geometry/ground_ribs.vtp
	../Geometry/ground_skull.vtp
	../Geometry/ground_spine.vtp
	../Geometry/hamate.vtp
	../Geometry/hamate_lvs.vtp
	../Geometry/hamate_rvs.vtp
	../Geometry/hand_1mc.vtp
	../Geometry/hand_2distph.vtp
	../Geometry/hand_2midph.vtp
	../Geometry/hand_2proxph.vtp
	../Geometry/hand_3distph.vtp
	../Geometry/hand_3midph.vtp
	../Geometry/hand_3proxph.vtp
	../Geometry/hand_4distph.vtp
	../Geometry/hand_4midph.vtp
	../Geometry/hand_4proxph.vtp
	../Geometry/hand_5distph.vtp
	../Geometry/hand_5midph.vtp
	../Geometry/hand_5proxph.vtp
	../Geometry/hand_thumbdist.vtp
	../Geometry/hand_thumbprox.vtp
	../Geometry/hat_jaw.vtp
	../Geometry/hat_ribs.vtp
	../Geometry/hat_skull.vtp
	../Geometry/hat_spine.vtp
	../Geometry/humerus.vtp
	../Geometry/humerus_lv.vtp
	../Geometry/humerus_rv.vtp
	../Geometry/index_distal_lvs.vtp
	../Geometry/index_distal_rvs.vtp
	../Geometry/index_medial_lvs.vtp
	../Geometry/index_medial_rvs.vtp
	../Geometry/index_proximal_lvs.vtp
	../Geometry/index_proximal_rvs.vtp
	../Geometry/jaw.vtp
	../Geometry/jaw_s.vtp
	../Geometry/l_bofoot.vtp
	../Geometry/l_femur.vtp
	../Geometry/l_fibula.vtp
	../Geometry/l_foot.vtp
	../Geometry/l_pat.vtp
	../Geometry/l_patella.vtp
	../Geometry/l_pelvis.vtp
	../Geometry/l_talus.vtp
	../Geometry/l_tibia.vtp
	../Geometry/line.vtp
	../Geometry/little_distal_lvs.vtp
	../Geometry/little_distal_rvs.vtp
	../Geometry/little_medial_lvs.vtp
	../Geometry/little_medial_rvs.vtp
	../Geometry/little_proximal_lvs.vtp
	../Geometry/little_proximal_rvs.vtp
	../Geometry/lumbar1.vtp
	../Geometry/lumbar1sm.vtp
	../Geometry/lumbar2.vtp
	../Geometry/lumbar2sm.vtp
	../Geometry/lumbar3.vtp
	../Geometry/lumbar3sm.vtp
	../Geometry/lumbar4.vtp
	../Geometry/lumbar4sm.vtp
	../Geometry/lumbar5.vtp
	../Geometry/lunate.vtp
	../Geometry/lunate_lvs.vtp
	../Geometry/lunate_rvs.vtp
	../Geometry/metacarpal1_lvs.vtp
	../Geometry/metacarpal1_rvs.vtp
	../Geometry/metacarpal2_lvs.vtp
	../Geometry/metacarpal2_rvs.vtp
	../Geometry/metacarpal3_lvs.vtp
	../Geometry/metacarpal3_rvs.vtp
	../Geometry/metacarpal4_lvs.vtp
	../Geometry/metacarpal4_rvs.vtp
	../Geometry/metacarpal5_lvs.vtp
	../Geometry/metacarpal5_rvs.vtp
	../Geometry/metatarsal1_lv.vtp
	../Geometry/metatarsal1_rv.vtp
	../Geometry/metatarsal2_lv.vtp
	../Geometry/metatarsal2_rv.vtp
	../Geometry/metatarsal3_lv.vtp
	../Geometry/metatarsal3_rv.vtp
	../Geometry/metatarsal4_lv.vtp
	../Geometry/metatarsal4_rv.vtp
	../Geometry/metatarsal5_lv.vtp
	../Geometry/metatarsal5_rv.vtp
	../Geometry/mid_phalanx2_lvs.vtp
	../Geometry/mid_phalanx2_rvs.vtp
	../Geometry/mid_phalanx3_lvs.vtp
	../Geometry/mid_phalanx3_rvs.vtp
	../Geometry/mid_phalanx4_lvs.vtp
	../Geometry/mid_phalanx4_rvs.vtp
	../Geometry/mid_phalanx5_lvs.vtp
	../Geometry/mid_phalanx5_rvs.vtp
	../Geometry/middle_distal_lvs.vtp
	../Geometry/middle_distal_rvs.vtp
	../Geometry/middle_medial_lvs.vtp
	../Geometry/middle_medial_rvs.vtp
	../Geometry/middle_proximal_lvs.vtp
	../Geometry/middle_proximal_rvs.vtp
	../Geometry/navicular_lv.vtp
	../Geometry/navicular_rv.vtp
	../Geometry/outline.vtp
	../Geometry/pat.vtp
	../Geometry/patella_l.vtp
	../Geometry/patella_r.vtp
	../Geometry/pelvis.vtp
	../Geometry/pelvis_lv.vtp
	../Geometry/pelvis_lvR.vtp
	../Geometry/pelvis_rv.vtp
	../Geometry/pelvis_rvR.vtp
	../Geometry/pisiform.vtp
	../Geometry/pisiform_lvs.vtp
	../Geometry/pisiform_rvs.vtp
	../Geometry/plane.vtp
	../Geometry/prox_phalanx1_lvs.vtp
	../Geometry/prox_phalanx1_rvs.vtp
	../Geometry/prox_phalanx2_lvs.vtp
	../Geometry/prox_phalanx2_rvs.vtp
	../Geometry/prox_phalanx3_lvs.vtp
	../Geometry/prox_phalanx3_rvs.vtp
	../Geometry/prox_phalanx4_lvs.vtp
	../Geometry/prox_phalanx4_rvs.vtp
	../Geometry/prox_phalanx5_lvs.vtp
	../Geometry/prox_phalanx5_rvs.vtp
	../Geometry/r_ham.vtp
	../Geometry/r_lun.vtp
	../Geometry/r_pis.vtp
	../Geometry/r_scaph.vtp
	../Geometry/r_triq.vtp
	../Geometry/r_trpzd.vtp
	../Geometry/r_trpzm.vtp
	../Geometry/radius.vtp
	../Geometry/radius_lv.vtp
	../Geometry/radius_rv.vtp
	../Geometry/ribcage.vtp
	../Geometry/ribcage_s.vtp
	../Geometry/ring_distal_lvs.vtp
	../Geometry/ring_distal_rvs.vtp
	../Geometry/ring_medial_lvs.vtp
	../Geometry/ring_medial_rvs.vtp
	../Geometry/ring_proximal_lvs.vtp
	../Geometry/ring_proximal_rvs.vtp
	../Geometry/sacrum.vtp
	../Geometry/scaphoid.vtp
	../Geometry/scaphoid_lvs.vtp
	../Geometry/scaphoid_rvs.vtp
	../Geometry/scapula.vtp
	../Geometry/scapula_lv.vtp
	../Geometry/scapula_lvsm.vtp
	../Geometry/scapula_rv.vtp
	../Geometry/scapula_rvsm.vtp
	../Geometry/sdfast_1seg_hand_fr_c_2mc.vtp
	../Geometry/sdfast_1seg_hand_fr_c_3mc.vtp
	../Geometry/sdfast_1seg_hand_fr_c_4mc.vtp
	../Geometry/sdfast_1seg_hand_fr_c_5mc.vtp
	../Geometry/sdfast_1seg_hand_fr_c_hamate.vtp
	../Geometry/sdfast_1seg_hand_fr_c_trapezium.vtp
	../Geometry/sdfast_1seg_hand_fr_c_trapezoid.vtp
	../Geometry/sdfastPISIFORMw.vtp
	../Geometry/sdfastSCAPHOIDw.vtp
	../Geometry/sdfastTRIQUETRALw.vtp
	../Geometry/second_cuneiform_lv.vtp
	../Geometry/second_cuneiform_rv.vtp
	../Geometry/skull.vtp
	../Geometry/skull_s.vtp
	../Geometry/sphere.vtp
	../Geometry/superquadric.vtp
	../Geometry/talus.vtp
	../Geometry/talus_lv.vtp
	../Geometry/talus_rv.vtp
	../Geometry/third_cuneiform_lv.vtp
	../Geometry/third_cuneiform_rv.vtp
	../Geometry/thoracic1.vtp
	../Geometry/thoracic1_s.vtp
	../Geometry/thoracic10.vtp
	../Geometry/thoracic10_s.vtp
	../Geometry/thoracic11.vtp
	../Geometry/thoracic11_s.vtp
	../Geometry/thoracic12.vtp
	../Geometry/thoracic12_s.vtp
	../Geometry/thoracic2.vtp
	../Geometry/thoracic2_s.vtp
	../Geometry/thoracic3.vtp
	../Geometry/thoracic3_s.vtp
	../Geometry/thoracic4.vtp
	../Geometry/thoracic4_s.vtp
	../Geometry/thoracic5.vtp
	../Geometry/thoracic5_s.vtp
	../Geometry/thoracic6.vtp
	../Geometry/thoracic6_s.vtp
	../Geometry/thoracic7.vtp
	../Geometry/thoracic7_s.vtp
	../Geometry/thoracic8.vtp
	../Geometry/thoracic8_s.vtp
	../Geometry/thoracic9.vtp
	../Geometry/thoracic9_s.vtp
	../Geometry/thorax.vtp
	../Geometry/thumb_distal_lvs.vtp
	../Geometry/thumb_distal_rvs.vtp
	../Geometry/thumb_proximal_lvs.vtp
	../Geometry/thumb_proximal_rvs.vtp
	../Geometry/thumbdist.vtp
	../Geometry/thumbprox.vtp
	../Geometry/tibia.vtp
	../Geometry/tibia_l.vtp
	../Geometry/tibia_r.vtp
	../Geometry/trapezium.vtp
	../Geometry/trapezium_lvs.vtp
	../Geometry/trapezium_rvs.vtp
	../Geometry/trapezoid.vtp
	../Geometry/trapezoid_lvs.vtp
	../Geometry/trapezoid_rvs.vtp
	../Geometry/treadmill.vtp
	../Geometry/triquetrum.vtp
	../Geometry/triquetrum_lvs.vtp
	../Geometry/triquetrum_rvs.vtp
	../Geometry/ulna.vtp
	../Geometry/ulna_lv.vtp
	../Geometry/ulna_rv.vtp
	../Geometry/unit_plane.obj
)

install( FILES ${GeometryList} DESTINATION lib/Geometry)