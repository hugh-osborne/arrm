#include "Sim.h"
#include "ConstraintRod.h"

ARRMSim::ARRMSim(double _timestep, double _sim_length, bool _visualisation) :
	timestep(_timestep), sim_length(_sim_length), skeletal_model_viz(_visualisation) {

    biceps_long = 0;
    brachioradialis = 1;
    triceps_long = 2;
    triceps_med = 3;
    delt_ant = 4;
    delt_med = 5;
    delt_post = 6;
    infraspinatus = 7;
    pec_maj = 8;
    lat = 9;
    fcr = 10;
    ecr = 11;

    update_vis_count = 1;
    update_vis_counter = 0;
    time = 0;

    string_length = 0.5;
    rod_length = 0.5;

    use_visualiser = true;
    reportForces = true;
    reportReactions = true;

    // by default, lock the wrist
    locked_wrist_pro_sup = true;
    locked_wrist_deviation = true;
    locked_wrist_flexion = true;
    start_angle_wrist_pro_sup = 0.0;
    start_angle_wrist_deviation = 0.0;
    start_angle_wrist_flexion = 0.0;

	visualiser_directory = std::string("");
}

void ARRMSim::setVisualiserDirectory(char *dir) {
	visualiser_directory = std::string(dir);
}

void ARRMSim::begin() {
    // Register Afferent Muscle Type so it can be read by the XML
    OpenSim::Object::registerType(OpenSim::Millard12EqMuscleWithAfferents());

    if (string_active)
        model = new OpenSim::Model("MoBL_ARMS_module2_4_onemuscle_afferent.osim");
    else
        model = new OpenSim::Model("MoBL_ARMS_module2_4_onemuscle_afferent_no_string.osim");

    if (use_visualiser) {
        model->setUseVisualizer(skeletal_model_viz);

        if (visualiser_directory != std::string(""))
            model->addVisualizerExectuableSearchDir(visualiser_directory);
    }

    model->updCoordinateSet().get("sternoclavicular_r2").setDefaultLocked(false);
    model->updCoordinateSet().get("sternoclavicular_r2").setDefaultValue(0.0);

    model->updCoordinateSet().get("sternoclavicular_r3").setDefaultLocked(false);
    model->updCoordinateSet().get("sternoclavicular_r3").setDefaultValue(0.0);

    model->updCoordinateSet().get("unrotscap_r3").setDefaultLocked(false);
    model->updCoordinateSet().get("unrotscap_r3").setDefaultValue(0.0);

    model->updCoordinateSet().get("unrotscap_r2").setDefaultLocked(false);
    model->updCoordinateSet().get("unrotscap_r2").setDefaultValue(0.0);

    model->updCoordinateSet().get("acromioclavicular_r2").setDefaultLocked(false);
    model->updCoordinateSet().get("acromioclavicular_r2").setDefaultValue(0.0);

    model->updCoordinateSet().get("acromioclavicular_r3").setDefaultLocked(false);
    model->updCoordinateSet().get("acromioclavicular_r3").setDefaultValue(0.0);

    model->updCoordinateSet().get("acromioclavicular_r1").setDefaultLocked(false);
    model->updCoordinateSet().get("acromioclavicular_r1").setDefaultValue(0.0);

    model->updCoordinateSet().get("unrothum_r1").setDefaultLocked(false);
    model->updCoordinateSet().get("unrothum_r1").setDefaultValue(0.0);

    model->updCoordinateSet().get("unrothum_r3").setDefaultLocked(false);
    model->updCoordinateSet().get("unrothum_r3").setDefaultValue(0.0);

    model->updCoordinateSet().get("unrothum_r2").setDefaultLocked(false);
    model->updCoordinateSet().get("unrothum_r2").setDefaultValue(0.0);

    model->updCoordinateSet().get("elv_angle").setDefaultLocked(false); //nothing
    model->updCoordinateSet().get("elv_angle").setDefaultValue(0.0);

    model->updCoordinateSet().get("shoulder_elv").setDefaultLocked(locked_shoulder_add_abd); // shoulder Adduction/abduction
    model->updCoordinateSet().get("shoulder_elv").setDefaultValue(start_angle_shoulder_add_abd);

    model->updCoordinateSet().get("shoulder1_r2").setDefaultLocked(false); // nothing
    model->updCoordinateSet().get("shoulder1_r2").setDefaultValue(0.0);

    model->updCoordinateSet().get("shoulder_rot").setDefaultLocked(locked_shoulder_twist); // shoulder y axis rotation
    model->updCoordinateSet().get("shoulder_rot").setDefaultValue(start_angle_shoulder_twist);

    model->updCoordinateSet().get("shoulder_ext").setDefaultLocked(locked_shoulder_ext_flex); //shoulder extension/flexion
    model->updCoordinateSet().get("shoulder_ext").setDefaultValue(start_angle_shoulder_ext_flex);

    model->updCoordinateSet().get("elbow_flexion").setDefaultLocked(locked_elbow);
    model->updCoordinateSet().get("elbow_flexion").setDefaultValue(start_angle_elbow); // 0.698132 = 40 degrees in radians

    model->updCoordinateSet().get("pro_sup").setDefaultLocked(locked_wrist_pro_sup);
    model->updCoordinateSet().get("pro_sup").setDefaultValue(start_angle_wrist_pro_sup);

    model->updCoordinateSet().get("deviation").setDefaultLocked(locked_wrist_deviation);
    model->updCoordinateSet().get("deviation").setDefaultValue(start_angle_wrist_deviation);

    model->updCoordinateSet().get("flexion").setDefaultLocked(locked_wrist_flexion);
    model->updCoordinateSet().get("flexion").setDefaultValue(start_angle_wrist_flexion);

    elbow_coord = (OpenSim::Coordinate*)&model->updCoordinateSet().get("elbow_flexion");
    shoulder_ext_coord = (OpenSim::Coordinate*)&model->updCoordinateSet().get("shoulder_ext");
    shoulder_abd_coord = (OpenSim::Coordinate*)&model->updCoordinateSet().get("shoulder_elv");
    shoulder_rot_coord = (OpenSim::Coordinate*)&model->updCoordinateSet().get("shoulder_rot");

    wrist_pro_sup_coord = (OpenSim::Coordinate*)&model->updCoordinateSet().get("pro_sup");
    wrist_deviation_coord = (OpenSim::Coordinate*)&model->updCoordinateSet().get("deviation");
    wrist_flexion_coord = (OpenSim::Coordinate*)&model->updCoordinateSet().get("flexion");

    elbow_point = (OpenSim::Body*)&model->updBodySet().get("ulna");

    anchor_joint = (OpenSim::CustomJoint*)&model->updJointSet().get("anchor_joint");
    radius = (OpenSim::Body*)&model->updBodySet().get("hand");

    _biceps_long = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("BIClong");
    _biceps_long->setupAfferents();

    _brachioradialis = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("BRD");
    _brachioradialis->setupAfferents();

    _triceps_long = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("TRIlong");
    _triceps_long->setupAfferents();

    _triceps_med = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("TRImed");
    _triceps_med->setupAfferents();

    _delt_ant = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("DELT1");
    _delt_ant->setupAfferents();

    _delt_med = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("DELT2");
    _delt_med->setupAfferents();

    _delt_post = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("DELT3");
    _delt_post->setupAfferents();

    _infraspinatus = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("INFSP");
    _infraspinatus->setupAfferents();

    _fcr = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("FCR");
    _fcr->setupAfferents();

    _ecr_l = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("ECRL");
    _ecr_l->setupAfferents();

    _pec_m_1 = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("PECM1");
    _pec_m_1->setupAfferents();

    _pec_m_2 = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("PECM2");
    _pec_m_2->setupAfferents();

    _pec_m_3 = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("PECM3");
    _pec_m_3->setupAfferents();

    _lat_1 = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("LAT1");
    _lat_1->setupAfferents();

    _lat_2 = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("LAT2");
    _lat_2->setupAfferents();

    _lat_3 = (OpenSim::Millard12EqMuscleWithAfferents*)&model->updMuscles().get("LAT3");
    _lat_3->setupAfferents();

    // Add a SimTK rope between the anchor_point on the hand and the tension_point

    anchor_point = (OpenSim::Body*)&model->updBodySet().get("anchor_point");

    if (string_active || rod_active || point_constraint_active) {
        tension_point = (OpenSim::Body*)&model->updBodySet().get("tension_point");

        if (string_active) {
            string_body = (OpenSim::Ligament*)&model->updForceSet().get("StringLig");
            string_body->setRestingLength(string_length);
        }

        if (rod_active) {
            OpenSim::ConstraintRod* cs = new OpenSim::ConstraintRod(*anchor_point, SimTK::Vec3(0.0, 0.0, 0.0), *tension_point, SimTK::Vec3(0.0, 0.0, 0.0), rod_length);
            model->addComponent(cs);
        }

        if (point_constraint_active) {
            point_constraint_point = (OpenSim::Body*)&model->updBodySet().get("point_constraint_point");
            OpenSim::PointConstraint* pcs = new OpenSim::PointConstraint(*radius, SimTK::Vec3(0.0, 0.0, 0.0), *point_constraint_point, SimTK::Vec3(0.0, 0.0, 0.0));
            model->addComponent(pcs);
        }
    }

    /*reporter = new OpenSim::ConsoleReporter();
    reporter->set_report_time_interval(0.01);
    reporter->addToReport(_biceps_long->getSpindle()->getOutput("primary_Ia"));
    reporter->addToReport(_biceps_long->getSpindle()->getOutput("secondary_II"));
    reporter->addToReport(_biceps_long->getGTO()->getOutput("gto_out"));
    reporter->addToReport(_biceps_long->getOutput("fiber_force"));
    reporter->addToReport(_biceps_long->getOutput("fiber_length"));
    reporter->addToReport(_biceps_long->getOutput("fiber_velocity"));
    reporter->addToReport(_biceps_long->getOutput("lpf_velocity"));
    reporter->addToReport(_biceps_long->getOutput("lpf_acceleration"));
    model->addComponent(reporter);

    table_reporter = new OpenSim::TableReporter();
    table_reporter->set_report_time_interval(0.001);
    table_reporter->addToReport(_biceps_long->getSpindle()->getOutput("primary_Ia"));
    table_reporter->addToReport(_biceps_long->getSpindle()->getOutput("secondary_II"));
    table_reporter->addToReport(_biceps_long->getGTO()->getOutput("gto_out"));
    table_reporter->addToReport(_biceps_long->getOutput("fiber_force"));
    table_reporter->addToReport(_biceps_long->getOutput("fiber_length"));
    table_reporter->addToReport(_biceps_long->getOutput("lpf_velocity"));
    table_reporter->addToReport(_biceps_long->getOutput("lpf_acceleration"));
    model->addComponent(table_reporter);*/

    force_reporter = new OpenSim::ForceReporter();
    force_reporter->setModel(*model);
    if (reportForces)
        model->addAnalysis(force_reporter);

    OpenSim::NeuralController* brain = new OpenSim::NeuralController();

    //brain->setActuators(model->updActuators());
    if (biceps_long_active) {
        brain->addActuator(*_biceps_long);
        biceps_long_activation[0] = new OpenSim::Constant(0.0);
        biceps_long_activation[1] = new OpenSim::Constant(0.0);
        biceps_long_activation[2] = new OpenSim::Constant(0.0);
        brain->associateFunctionWithAlphaInput("BIClong", biceps_long_activation[0]);
        brain->associateFunctionWithBetaInput("BIClong", biceps_long_activation[1]);
        brain->associateFunctionWithGammaInput("BIClong", biceps_long_activation[2]);
        brain->prescribeControlForActuator("BIClong");
    }

    if (brachioradialis_active) {
        brain->addActuator(*_brachioradialis);
        brachioradialis_activation[0] = new OpenSim::Constant(0.0);
        brachioradialis_activation[1] = new OpenSim::Constant(0.0);
        brachioradialis_activation[2] = new OpenSim::Constant(0.0);
        brain->associateFunctionWithAlphaInput("BRD", brachioradialis_activation[0]);
        brain->associateFunctionWithBetaInput("BRD", brachioradialis_activation[1]);
        brain->associateFunctionWithGammaInput("BRD", brachioradialis_activation[2]);
        brain->prescribeControlForActuator("BRD");
    }

    if (triceps_long_active) {
        brain->addActuator(*_triceps_long);
        triceps_long_activation[0] = new OpenSim::Constant(0.0);
        triceps_long_activation[1] = new OpenSim::Constant(0.0);
        triceps_long_activation[2] = new OpenSim::Constant(0.0);
        brain->associateFunctionWithAlphaInput("TRIlong", triceps_long_activation[0]);
        brain->associateFunctionWithBetaInput("TRIlong", triceps_long_activation[1]);
        brain->associateFunctionWithGammaInput("TRIlong", triceps_long_activation[2]);
        brain->prescribeControlForActuator("TRIlong");
    }

    if (triceps_med_active) {
        brain->addActuator(*_triceps_med);
        triceps_med_activation[0] = new OpenSim::Constant(0.0);
        triceps_med_activation[1] = new OpenSim::Constant(0.0);
        triceps_med_activation[2] = new OpenSim::Constant(0.0);
        brain->associateFunctionWithAlphaInput("TRImed", triceps_med_activation[0]);
        brain->associateFunctionWithBetaInput("TRImed", triceps_med_activation[1]);
        brain->associateFunctionWithGammaInput("TRImed", triceps_med_activation[2]);
        brain->prescribeControlForActuator("TRImed");
    }

    if (delt_ant_active) {
        brain->addActuator(*_delt_ant);
        delt_ant_activation[0] = new OpenSim::Constant(0.0);
        delt_ant_activation[1] = new OpenSim::Constant(0.0);
        delt_ant_activation[2] = new OpenSim::Constant(0.0);

        brain->associateFunctionWithAlphaInput("DELT1", delt_ant_activation[0]);
        brain->associateFunctionWithBetaInput("DELT1", delt_ant_activation[1]);
        brain->associateFunctionWithGammaInput("DELT1", delt_ant_activation[2]);
        brain->prescribeControlForActuator("DELT1");
    }

    if (delt_med_active) {
        brain->addActuator(*_delt_med);
        delt_med_activation[0] = new OpenSim::Constant(0.0);
        delt_med_activation[1] = new OpenSim::Constant(0.0);
        delt_med_activation[2] = new OpenSim::Constant(0.0);

        brain->associateFunctionWithAlphaInput("DELT2", delt_med_activation[0]);
        brain->associateFunctionWithBetaInput("DELT2", delt_med_activation[1]);
        brain->associateFunctionWithGammaInput("DELT2", delt_med_activation[2]);
        brain->prescribeControlForActuator("DELT2");
    }

    if (delt_post_active) {
        brain->addActuator(*_delt_post);
        delt_post_activation[0] = new OpenSim::Constant(0.0);
        delt_post_activation[1] = new OpenSim::Constant(0.0);
        delt_post_activation[2] = new OpenSim::Constant(0.0);

        brain->associateFunctionWithAlphaInput("DELT3", delt_post_activation[0]);
        brain->associateFunctionWithBetaInput("DELT3", delt_post_activation[1]);
        brain->associateFunctionWithGammaInput("DELT3", delt_post_activation[2]);
        brain->prescribeControlForActuator("DELT3");
    }

    if (infraspinatus_active) {
        brain->addActuator(*_infraspinatus);
        infraspinatus_activation[0] = new OpenSim::Constant(0.0);
        infraspinatus_activation[1] = new OpenSim::Constant(0.0);
        infraspinatus_activation[2] = new OpenSim::Constant(0.0);

        brain->associateFunctionWithAlphaInput("INFSP", infraspinatus_activation[0]);
        brain->associateFunctionWithBetaInput("INFSP", infraspinatus_activation[1]);
        brain->associateFunctionWithGammaInput("INFSP", infraspinatus_activation[2]);
        brain->prescribeControlForActuator("INFSP");
    }

    if (pec_maj_active) {
        brain->addActuator(*_pec_m_2);
        brain->addActuator(*_pec_m_1);
        brain->addActuator(*_pec_m_3);
        pec_m_2_activation[0] = new OpenSim::Constant(0.0);
        pec_m_2_activation[1] = new OpenSim::Constant(0.0);
        pec_m_2_activation[2] = new OpenSim::Constant(0.0);

        brain->associateFunctionWithAlphaInput("PECM2", pec_m_2_activation[0]);
        brain->associateFunctionWithBetaInput("PECM2", pec_m_2_activation[1]);
        brain->associateFunctionWithGammaInput("PECM2", pec_m_2_activation[2]);
        brain->prescribeControlForActuator("PECM2");
        brain->associateFunctionWithAlphaInput("PECM1", pec_m_2_activation[0]);
        brain->associateFunctionWithBetaInput("PECM1", pec_m_2_activation[1]);
        brain->associateFunctionWithGammaInput("PECM1", pec_m_2_activation[2]);
        brain->prescribeControlForActuator("PECM1");
        brain->associateFunctionWithAlphaInput("PECM3", pec_m_2_activation[0]);
        brain->associateFunctionWithBetaInput("PECM3", pec_m_2_activation[1]);
        brain->associateFunctionWithGammaInput("PECM3", pec_m_2_activation[2]);
        brain->prescribeControlForActuator("PECM3");
    }

    if (lat_active) {
        brain->addActuator(*_lat_2);
        brain->addActuator(*_lat_1);
        brain->addActuator(*_lat_3);
        lat_2_activation[0] = new OpenSim::Constant(0.0);
        lat_2_activation[1] = new OpenSim::Constant(0.0);
        lat_2_activation[2] = new OpenSim::Constant(0.0);

        brain->associateFunctionWithAlphaInput("LAT2", lat_2_activation[0]);
        brain->associateFunctionWithBetaInput("LAT2", lat_2_activation[1]);
        brain->associateFunctionWithGammaInput("LAT2", lat_2_activation[2]);
        brain->prescribeControlForActuator("LAT2");
        brain->associateFunctionWithAlphaInput("LAT1", lat_2_activation[0]);
        brain->associateFunctionWithBetaInput("LAT1", lat_2_activation[1]);
        brain->associateFunctionWithGammaInput("LAT1", lat_2_activation[2]);
        brain->prescribeControlForActuator("LAT1");
        brain->associateFunctionWithAlphaInput("LAT3", lat_2_activation[0]);
        brain->associateFunctionWithBetaInput("LAT3", lat_2_activation[1]);
        brain->associateFunctionWithGammaInput("LAT3", lat_2_activation[2]);
        brain->prescribeControlForActuator("LAT3");
    }

    if (fcr_active) {
        brain->addActuator(*_fcr);
        fcr_activation[0] = new OpenSim::Constant(0.0);
        fcr_activation[1] = new OpenSim::Constant(0.0);
        fcr_activation[2] = new OpenSim::Constant(0.0);

        brain->associateFunctionWithAlphaInput("FCR", fcr_activation[0]);
        brain->associateFunctionWithBetaInput("FCR", fcr_activation[1]);
        brain->associateFunctionWithGammaInput("FCR", fcr_activation[2]);
        brain->prescribeControlForActuator("FCR");
    }

    if (ecr_active) {
        brain->addActuator(*_ecr_l);
        ecr_l_activation[0] = new OpenSim::Constant(0.0);
        ecr_l_activation[1] = new OpenSim::Constant(0.0);
        ecr_l_activation[2] = new OpenSim::Constant(0.0);

        brain->associateFunctionWithAlphaInput("ECRL", ecr_l_activation[0]);
        brain->associateFunctionWithBetaInput("ECRL", ecr_l_activation[1]);
        brain->associateFunctionWithGammaInput("ECRL", ecr_l_activation[2]);
        brain->prescribeControlForActuator("ECRL");
    }

    model->addController(brain);

    // Reaction Forces on Wrist

    jr = new OpenSim::JointReaction(model);

    OpenSim::Array<std::string> joints;
    joints.append("wrist_hand");
    joints.append("point_constraint_joint");
    joints.append("anchor_joint");
    joints.append("tension_joint");

    OpenSim::Array<std::string> on_body;
    on_body.append("child");
    on_body.append("child");
    on_body.append("child");
    on_body.append("child");

    OpenSim::Array<std::string> in_frame;
    in_frame.append("ground");
    in_frame.append("ground");
    in_frame.append("ground");
    in_frame.append("ground");

    jr->setJointNames(joints);
    jr->setOnBody(on_body);
    jr->setInFrame(in_frame);

    if (reportReactions)
        model->addAnalysis(jr);

    const SimTK::Vec3 a = SimTK::Vec3(0.0, 0.0, 0.0);
    const SimTK::Vec3 b = SimTK::Vec3(0.0, 0.0, 0.0);

    SimTK::State& state = model->initSystem();

    if (use_visualiser) {
        // Configure the visualizer.
        //model->updMatterSubsystem().setShowDefaultGeometry(true);
       SimTK::Visualizer& viz = model->updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.SolidColor);
        viz.setBackgroundColor(SimTK::Black);
    }

    model->equilibrateMuscles(state);

    integrator = new SimTK::RungeKutta3Integrator(model->getSystem());
    integrator->setFixedStepSize(timestep);
    integrator->setAccuracy(1e-003);
    integrator->setAllowInterpolation(true);
    integrator->setProjectInterpolatedStates(true);
    ts = new SimTK::TimeStepper(model->getSystem(), *integrator);
    ts->initialize(state);
    ts->setReportAllSignificantStates(false);
    integrator->setReturnEveryInternalStep(true);
    integrator->setProjectEveryStep(false);
    std::cout << "Starting Simulation...\n";

    initState = &state;

    update_vis_count = 1;
    update_vis_counter = 0;

    time = 0;

    jr->begin(ts->getState());

    force_reporter->begin(ts->getState());
}

double ARRMSim::getHandPositionX() {
    return anchor_point->getPositionInGround(ts->getState())[0];
}

double ARRMSim::getHandPositionY() {
    return anchor_point->getPositionInGround(ts->getState())[1];
}

double ARRMSim::getHandPositionZ() {
    return anchor_point->getPositionInGround(ts->getState())[2];
}

double ARRMSim::getElbowPositionX() {
    return elbow_point->getPositionInGround(ts->getState())[0];
}

double ARRMSim::getElbowPositionY() {
    return elbow_point->getPositionInGround(ts->getState())[1];
}

double ARRMSim::getElbowPositionZ() {
    return elbow_point->getPositionInGround(ts->getState())[2];
}

double ARRMSim::getElbowAngle() {
    return elbow_coord->getValue(ts->getState());
}

double ARRMSim::getShoulderAbdAngle() {
    return shoulder_abd_coord->getValue(ts->getState());
}

double ARRMSim::getShoulderTwistAngle() {
    return shoulder_rot_coord->getValue(ts->getState());
}

double ARRMSim::getShoulderExtAngle() {
    return shoulder_ext_coord->getValue(ts->getState());
}

double ARRMSim::getWristProSupAngle() {
    return wrist_pro_sup_coord->getValue(ts->getState());
}

double ARRMSim::getWristDeviationAngle() {
    return wrist_deviation_coord->getValue(ts->getState());
}

double ARRMSim::getWristFlexionAngle() {
    return wrist_flexion_coord->getValue(ts->getState());
}

double ARRMSim::getHandAccelX() {
    return anchor_point->getAccelerationInGround(ts->getState())[1][0];
}

double ARRMSim::getHandAccelY() {
    return anchor_point->getAccelerationInGround(ts->getState())[1][1];
}

double ARRMSim::getHandAccelZ() {
    return anchor_point->getAccelerationInGround(ts->getState())[1][2];
}

void ARRMSim::update() {
    time += timestep;
    update_vis_counter++;
    std::cout << "(" << ts->getState().getTime() << ") " << integrator->getSuccessfulStepStatusString(ts->stepTo(time)) << "                  \r" << std::flush;
    states.append(ts->getState());

    model->realizeReport(ts->getState());

    if (update_vis_counter > update_vis_count && skeletal_model_viz) {
        update_vis_counter = 0;
        if(use_visualiser)
            model->getVisualizer().show(ts->getState());
    }

    jr->step(ts->getState(), 1);
    force_reporter->step(ts->getState(), 1);
}

void ARRMSim::end() {
    jr->end(ts->getState());
    jr->printResults("jointreaction"); 

    force_reporter->end(ts->getState());
    force_reporter->printResults("forces");

    /*auto table = table_reporter->getTable();

    std::ofstream dataFile;
    dataFile.open("output.txt");
    for (int i = 0; i < table.getNumRows(); i++) {
        for (int j = 0; j < 7; j++) { // Currently recording 8 metrics...
            dataFile << table.getRowAtIndex(i).getAsVector()[j] << "," << std::flush;
        }
        dataFile << "\n" << std::flush;
    }
    dataFile.close();*/
}

void ARRMSim::setMuscleActivity(unsigned int muscle, double alpha, double beta, double gamma) {
    if (biceps_long_active && muscle == biceps_long) {
        biceps_long_activation[0]->setValue(alpha);
        biceps_long_activation[1]->setValue(beta);
        biceps_long_activation[2]->setValue(gamma);
    }

    if (brachioradialis_active && muscle == brachioradialis) {
        brachioradialis_activation[0]->setValue(alpha);
        brachioradialis_activation[1]->setValue(beta);
        brachioradialis_activation[2]->setValue(gamma);
    }

    if (triceps_long_active && muscle == triceps_long) {
        triceps_long_activation[0]->setValue(alpha);
        triceps_long_activation[1]->setValue(beta);
        triceps_long_activation[2]->setValue(gamma);
    }

    if (triceps_med_active && muscle == triceps_med) {
        triceps_med_activation[0]->setValue(alpha);
        triceps_med_activation[1]->setValue(beta);
        triceps_med_activation[2]->setValue(gamma);
    }

    if (delt_ant_active && muscle == delt_ant) {
        delt_ant_activation[0]->setValue(alpha);
        delt_ant_activation[1]->setValue(beta);
        delt_ant_activation[2]->setValue(gamma);
    }

    if (delt_med_active && muscle == delt_med) {
        delt_med_activation[0]->setValue(alpha);
        delt_med_activation[1]->setValue(beta);
        delt_med_activation[2]->setValue(gamma);
    }

    if (delt_post_active && muscle == delt_post) {
        delt_post_activation[0]->setValue(alpha);
        delt_post_activation[1]->setValue(beta);
        delt_post_activation[2]->setValue(gamma);
    }

    if (infraspinatus_active && muscle == infraspinatus) {
        infraspinatus_activation[0]->setValue(alpha);
        infraspinatus_activation[1]->setValue(beta);
        infraspinatus_activation[2]->setValue(gamma);
    }

    if (pec_maj_active && muscle == pec_maj) {
        pec_m_2_activation[0]->setValue(alpha);
        pec_m_2_activation[1]->setValue(beta);
        pec_m_2_activation[2]->setValue(gamma);
    }

    if (lat_active && muscle == lat) {
        lat_2_activation[0]->setValue(alpha);
        lat_2_activation[1]->setValue(beta);
        lat_2_activation[2]->setValue(gamma);
    }

    if (fcr_active && muscle == fcr) {
        fcr_activation[0]->setValue(alpha);
        fcr_activation[1]->setValue(beta);
        fcr_activation[2]->setValue(gamma);
    }

    if (ecr_active && muscle == ecr) {
        ecr_l_activation[0]->setValue(alpha);
        ecr_l_activation[1]->setValue(beta);
        ecr_l_activation[2]->setValue(gamma);
    }
}

double ARRMSim::getMuscleIa(unsigned int muscle) {
    if (biceps_long_active && muscle == biceps_long) {
        return _biceps_long->getSpindle()->getIaOutput(ts->getState());
    }

    if (brachioradialis_active && muscle == brachioradialis) {
        return _brachioradialis->getSpindle()->getIaOutput(ts->getState());
    }

    if (triceps_long_active && muscle == triceps_long) {
        return _triceps_long->getSpindle()->getIaOutput(ts->getState());
    }

    if (triceps_med_active && muscle == triceps_med) {
        return _triceps_med->getSpindle()->getIaOutput(ts->getState());
    }

    if (delt_ant_active && muscle == delt_ant) {
        return _delt_ant->getSpindle()->getIaOutput(ts->getState());
    }

    if (delt_med_active && muscle == delt_med) {
        return _delt_med->getSpindle()->getIaOutput(ts->getState());
    }

    if (delt_post_active && muscle == delt_post) {
        return _delt_post->getSpindle()->getIaOutput(ts->getState());
    }

    if (infraspinatus_active && muscle == infraspinatus) {
        return _infraspinatus->getSpindle()->getIaOutput(ts->getState());
    }

    if (pec_maj_active && muscle == pec_maj) {
        return _pec_m_2->getSpindle()->getIaOutput(ts->getState());
    }

    if (lat_active && muscle == lat) {
        return _lat_2->getSpindle()->getIaOutput(ts->getState());
    }

    if (fcr_active && muscle == fcr) {
        return _fcr->getSpindle()->getIaOutput(ts->getState());
    }

    if (ecr_active && muscle == ecr) {
        return _ecr_l->getSpindle()->getIaOutput(ts->getState());
    }
}

double ARRMSim::getMuscleIb(unsigned int muscle) {
    if (biceps_long_active && muscle == biceps_long) {
        return _biceps_long->getGTO()->getGTOout(ts->getState());
    }

    if (brachioradialis_active && muscle == brachioradialis) {
        return _brachioradialis->getGTO()->getGTOout(ts->getState());
    }

    if (triceps_long_active && muscle == triceps_long) {
        return _triceps_long->getGTO()->getGTOout(ts->getState());
    }

    if (triceps_med_active && muscle == triceps_med) {
        return _triceps_med->getGTO()->getGTOout(ts->getState());
    }

    if (delt_ant_active && muscle == delt_ant) {
        return _delt_ant->getGTO()->getGTOout(ts->getState());
    }

    if (delt_med_active && muscle == delt_med) {
        return _delt_med->getGTO()->getGTOout(ts->getState());
    }

    if (delt_post_active && muscle == delt_post) {
        return _delt_post->getGTO()->getGTOout(ts->getState());
    }

    if (infraspinatus_active && muscle == infraspinatus) {
        return _infraspinatus->getGTO()->getGTOout(ts->getState());
    }

    if (pec_maj_active && muscle == pec_maj) {
        return _pec_m_2->getGTO()->getGTOout(ts->getState());
    }

    if (lat_active && muscle == lat) {
        return _lat_2->getGTO()->getGTOout(ts->getState());
    }

    if (fcr_active && muscle == fcr) {
        return _fcr->getGTO()->getGTOout(ts->getState());
    }

    if (ecr_active && muscle == ecr) {
        return _ecr_l->getGTO()->getGTOout(ts->getState());
    }
}

double ARRMSim::getMuscleII(unsigned int muscle) {
    if (biceps_long_active && muscle == biceps_long) {
        return _biceps_long->getSpindle()->getIIOutput(ts->getState());
    }

    if (brachioradialis_active && muscle == brachioradialis) {
        return _brachioradialis->getSpindle()->getIIOutput(ts->getState());
    }

    if (triceps_long_active && muscle == triceps_long) {
        return _triceps_long->getSpindle()->getIIOutput(ts->getState());
    }

    if (triceps_med_active && muscle == triceps_med) {
        return _triceps_med->getSpindle()->getIIOutput(ts->getState());
    }

    if (delt_ant_active && muscle == delt_ant) {
        return _delt_ant->getSpindle()->getIIOutput(ts->getState());
    }

    if (delt_med_active && muscle == delt_med) {
        return _delt_med->getSpindle()->getIIOutput(ts->getState());
    }

    if (delt_post_active && muscle == delt_post) {
        return _delt_post->getSpindle()->getIIOutput(ts->getState());
    }

    if (infraspinatus_active && muscle == infraspinatus) {
        return _infraspinatus->getSpindle()->getIIOutput(ts->getState());
    }

    if (pec_maj_active && muscle == pec_maj) {
        return _pec_m_2->getSpindle()->getIIOutput(ts->getState());
    }

    if (lat_active && muscle == lat) {
        return _lat_2->getSpindle()->getIIOutput(ts->getState());
    }

    if (fcr_active && muscle == fcr) {
        return _fcr->getSpindle()->getIIOutput(ts->getState());
    }

    if (ecr_active && muscle == ecr) {
        return _ecr_l->getSpindle()->getIIOutput(ts->getState());
    }
}