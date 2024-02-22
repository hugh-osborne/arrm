#ifndef OPENSIM_CONSTRAINT_ROD_H_
#define OPENSIM_CONSTRAINT_ROD_H_

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/OpenSim.h>

namespace OpenSim {

    class ConstraintRod : public ModelComponent {
        OpenSim_DECLARE_CONCRETE_OBJECT(ConstraintRod, ModelComponent);
    public:
        OpenSim_DECLARE_PROPERTY(location_body_1, SimTK::Vec3,
            "Location of the point in first body specified in body1 "
            "reference frame.");
        OpenSim_DECLARE_PROPERTY(location_body_2, SimTK::Vec3,
            "Location of the point in second body specified in body2 "
            "reference frame.");
        OpenSim_DECLARE_PROPERTY(constant_distance, double, "constant distance "
            "to be rigidly maintained between the two points "
            "fixed on each body.");

        OpenSim_DECLARE_SOCKET(body_1, PhysicalFrame,
            "The first body participating in this constraint.");
        OpenSim_DECLARE_SOCKET(body_2, PhysicalFrame,
            "The second body participating in this constraint.");

    public:
        ConstraintRod();
        ConstraintRod(
            const PhysicalFrame& body1, const SimTK::Vec3& locationBody1,
            const PhysicalFrame& body2, const SimTK::Vec3& locationBody2,
            const double& distance);

        virtual ~ConstraintRod();

        void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    private:
        /** Construct ConstantDistanceConstraint's properties */
        void constructProperties();

        void setNull();

    };

}

#endif