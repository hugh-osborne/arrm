#include "ConstraintRod.h"

#include <simbody/internal/MobilizedBody.h>
#include <simbody/internal/Constraint_Rod.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ConstraintRod::~ConstraintRod()
{
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
/* Default constructor.
 */
ConstraintRod::ConstraintRod() :
    ModelComponent()
{
    setNull();
    constructProperties();
}

/*
 * Convenience Constructor.
*/
ConstraintRod::ConstraintRod(
    const PhysicalFrame& body1, const SimTK::Vec3& locationBody1,
    const PhysicalFrame& body2, const SimTK::Vec3& locationBody2,
    const double& distance) : ModelComponent()
{
    setNull();
    constructProperties();

    connectSocket_body_1(body1);
    connectSocket_body_2(body2);
    set_location_body_1(locationBody1);
    set_location_body_2(locationBody2);
    set_constant_distance(distance);
}

void ConstraintRod::setNull()
{
    setAuthors("Matt S. DeMers. Extended by Hugh Osborne");
}

void ConstraintRod::constructProperties()
{
    //Default location and orientation (rotation sequence)
    SimTK::Vec3 origin(0.0, 0.0, 0.0);

    constructProperty_location_body_1(origin);
    constructProperty_location_body_2(origin);

    // Constant distance between points
    constructProperty_constant_distance(SimTK::NaN);
}

void ConstraintRod::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // Get underlying mobilized bodies
    const PhysicalFrame& f1 = getConnectee<PhysicalFrame>("body_1");
    const PhysicalFrame& f2 = getConnectee<PhysicalFrame>("body_2");

    SimTK::MobilizedBody b1 = f1.getMobilizedBody();
    SimTK::MobilizedBody b2 = f2.getMobilizedBody();

    // Now create a Simbody Constraint::Rod
    SimTK::Constraint::Rod simtkRod(b1, get_location_body_1(),
        b2, get_location_body_2(),
        get_constant_distance());

    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    //assignConstraintIndex(simtkRod.getConstraintIndex());

}