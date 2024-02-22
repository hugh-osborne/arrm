#ifndef ARRM_SIM_H
#define ARRM_SIM_H

#include <OpenSim/OpenSim.h>

#include <fstream>

#include "Afferent/Millard12EqMuscleWithAfferents.h"
#include "NeuralController.h"


class ARRMSim {
private:
	bool skeletal_model_viz;// = true;
	double sim_length;// = 0.5;
	double timestep;// = 0.0002;

	OpenSim::Arrow* forceArrow;
	OpenSim::JointReaction* jr;
	OpenSim::ForceReporter* force_reporter;

	OpenSim::CustomJoint* anchor_joint;
	OpenSim::Body* radius;

	OpenSim::Body* anchor_point;
	OpenSim::Body* tension_point;
	OpenSim::Body* point_constraint_point;
	OpenSim::Ligament* string_body;
	OpenSim::Body* elbow_point;

	OpenSim::Coordinate* shoulder_ext_coord;
	OpenSim::Coordinate* shoulder_abd_coord;
	OpenSim::Coordinate* shoulder_rot_coord;
	OpenSim::Coordinate* elbow_coord;
	OpenSim::Coordinate* wrist_pro_sup_coord;
	OpenSim::Coordinate* wrist_deviation_coord;
	OpenSim::Coordinate* wrist_flexion_coord;

	OpenSim::Millard12EqMuscleWithAfferents* _biceps_long;
	OpenSim::Millard12EqMuscleWithAfferents* _brachioradialis;
	OpenSim::Millard12EqMuscleWithAfferents* _triceps_long;
	OpenSim::Millard12EqMuscleWithAfferents* _triceps_med;
	OpenSim::Millard12EqMuscleWithAfferents* _delt_ant;
	OpenSim::Millard12EqMuscleWithAfferents* _delt_med;
	OpenSim::Millard12EqMuscleWithAfferents* _delt_post;
	OpenSim::Millard12EqMuscleWithAfferents* _infraspinatus;
	OpenSim::Millard12EqMuscleWithAfferents* _fcr;
	OpenSim::Millard12EqMuscleWithAfferents* _ecr_l;
	OpenSim::Millard12EqMuscleWithAfferents* _pec_m_1;
	OpenSim::Millard12EqMuscleWithAfferents* _pec_m_2;
	OpenSim::Millard12EqMuscleWithAfferents* _pec_m_3;
	OpenSim::Millard12EqMuscleWithAfferents* _lat_1;
	OpenSim::Millard12EqMuscleWithAfferents* _lat_2;
	OpenSim::Millard12EqMuscleWithAfferents* _lat_3;
	OpenSim::Constant* biceps_long_activation[3];
	OpenSim::Constant* brachioradialis_activation[3];
	OpenSim::Constant* triceps_long_activation[3];
	OpenSim::Constant* triceps_med_activation[3];
	OpenSim::Constant* delt_ant_activation[3];
	OpenSim::Constant* delt_med_activation[3];
	OpenSim::Constant* delt_post_activation[3];
	OpenSim::Constant* infraspinatus_activation[3];
	OpenSim::Constant* fcr_activation[3];
	OpenSim::Constant* ecr_l_activation[3];
	OpenSim::Constant* pec_m_1_activation;
	OpenSim::Constant* pec_m_2_activation[3];
	OpenSim::Constant* pec_m_3_activation;
	OpenSim::Constant* lat_1_activation;
	OpenSim::Constant* lat_2_activation[3];
	OpenSim::Constant* lat_3_activation;

	OpenSim::StatesTrajectory states;
	SimTK::RungeKutta3Integrator *integrator;
	SimTK::TimeStepper *ts;

	OpenSim::TableReporter* table_reporter;
	OpenSim::ConsoleReporter* reporter;

	unsigned int update_vis_count;
	unsigned int update_vis_counter;

	double time;

	OpenSim::Model *model;
	SimTK::State *initState;
	
	std::string visualiser_directory;

public:
	bool biceps_long_active;
	bool brachioradialis_active;
	bool triceps_long_active;
	bool triceps_med_active;
	bool delt_ant_active;
	bool delt_med_active;
	bool delt_post_active;
	bool infraspinatus_active;
	bool pec_maj_active;
	bool lat_active;
	bool fcr_active;
	bool ecr_active;

	bool string_active;
	bool rod_active;
	bool point_constraint_active;

	double string_length;
	double rod_length;

	bool locked_shoulder_add_abd;
	double start_angle_shoulder_add_abd;

	bool locked_shoulder_ext_flex;
	double start_angle_shoulder_ext_flex;

	bool locked_shoulder_twist;
	double start_angle_shoulder_twist;

	bool locked_wrist_pro_sup;
	double start_angle_wrist_pro_sup;

	bool locked_wrist_deviation;
	double start_angle_wrist_deviation;

	bool locked_wrist_flexion;
	double start_angle_wrist_flexion;

	bool locked_elbow;
	double start_angle_elbow;

	bool use_visualiser;
	bool reportForces;
	bool reportReactions;

	unsigned int biceps_long;
	unsigned int brachioradialis;
	unsigned int triceps_long;
	unsigned int triceps_med;
	unsigned int delt_ant;
	unsigned int delt_med;
	unsigned int delt_post;
	unsigned int infraspinatus;
	unsigned int pec_maj;
	unsigned int lat;
	unsigned int fcr;
	unsigned int ecr;

public:
	ARRMSim(double timestep, double sim_length, bool visualisation);
	
	void setVisualiserDirectory(char *dir);

	void setMuscleActivity(unsigned int muscle, double alpha, double beta, double gamma);
	double getMuscleIa(unsigned int muscle);
	double getMuscleIb(unsigned int muscle);
	double getMuscleII(unsigned int muscle);

	double getHandPositionX();
	double getHandPositionY();
	double getHandPositionZ();

	double getElbowPositionX();
	double getElbowPositionY();
	double getElbowPositionZ();

	double getElbowAngle();
	double getShoulderAbdAngle();
	double getShoulderTwistAngle();
	double getShoulderExtAngle();
	double getWristProSupAngle();
	double getWristDeviationAngle();
	double getWristFlexionAngle();

	double getHandAccelX();
	double getHandAccelY();
	double getHandAccelZ();

	void begin();
	void update();
	void end();
};

#endif