#include "../arrmlib/Sim.h"

int main() {
    double timestep = 0.0002;
    double simetime = 0.5;
    ARRMSim sim(timestep, simetime, true);
    
    sim.biceps_long_active = true;
    sim.brachioradialis_active = true;
    sim.triceps_long_active = true;
    sim.triceps_med_active = true;
    sim.delt_ant_active = true;
    sim.delt_med_active = true;
    sim.delt_post_active = true;
    sim.infraspinatus_active = true;
    sim.pec_maj_active = true;
    sim.lat_active = true;
    sim.fcr_active = false;
    sim.ecr_active = false;

    sim.string_active = false;
    sim.string_length = 1.55;
    sim.rod_active = false;
    sim.rod_length = 0.37;
    sim.point_constraint_active = false;

    sim.locked_shoulder_add_abd = false;
    sim.start_angle_shoulder_add_abd = 0.0;

    sim.locked_shoulder_ext_flex = false;
    sim.start_angle_shoulder_ext_flex = 0.0;

    sim.locked_shoulder_twist = false;
    sim.start_angle_shoulder_twist = -0.698132;

    sim.locked_elbow = false;
    sim.start_angle_elbow = 0.698132*2;
    
    sim.begin();

    double init_hand_x = sim.getHandPositionX();
    double init_hand_y = sim.getHandPositionY();
    double init_hand_z = sim.getHandPositionZ();

    double init_shoulder_abd_value = sim.getShoulderAbdAngle();
    double init_shoulder_twist_value = sim.getShoulderTwistAngle();

    for (int i = 0; i< int(simetime / timestep); i++) {

        double a_biceps = 0.0;
        double a_triceps = 0.0;
        double a_fcr = 0.0;
        double a_ecr = 0.0;
        double a_pec = 0.0;
        double a_lat = 0.0;
        double a_delt_p = 0.0;
        double a_delt_m = 0.0;
        double a_delt_a = 0.0;


        double biceps_vol = 0;
        double brach_vol = 0.0;
        double triceps_vol = 0.0;
        double triceps_med_vol = 0;
        double fcr_vol = 0.0;
        double ecr_vol = 0.0;
        double delt_ant_vol = 0.0;
        double delt_med_vol = 0.0;
        double delt_post_vol = 0.0;
        double lat_vol = 0.0;
        double pec_vol = 0.0;

        sim.setMuscleActivity(sim.biceps_long, a_biceps + biceps_vol, 0.0, 0.0);
        sim.setMuscleActivity(sim.brachioradialis, a_biceps + brach_vol, 0.0, 0.0);
        sim.setMuscleActivity(sim.triceps_long, a_triceps + triceps_vol, 0.0, 0.0);
        sim.setMuscleActivity(sim.triceps_med, a_triceps + triceps_med_vol, 0.0, 0.0);
        sim.setMuscleActivity(sim.fcr, a_fcr + fcr_vol, 0.0, 0.0);
        sim.setMuscleActivity(sim.ecr, a_ecr + ecr_vol, 0.0, 0.0);
        sim.setMuscleActivity(sim.delt_ant, 200.0 + delt_ant_vol, 0.0, 0.0);
        sim.setMuscleActivity(sim.delt_med, a_delt_m + delt_med_vol, 0.0, 0.0);
        sim.setMuscleActivity(sim.delt_post, a_delt_p + delt_post_vol, 0.0, 0.0);
        sim.setMuscleActivity(sim.infraspinatus, 0, 0.0, 0.0);
        sim.setMuscleActivity(sim.lat, a_lat + 0.0, 0.0, 0.0);
        sim.setMuscleActivity(sim.pec_maj, a_pec + 0.0, 0.0, 0.0);
        sim.update();
    } 

    sim.end();

    return 0;
};