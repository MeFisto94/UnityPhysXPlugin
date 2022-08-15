%include PxVehicleTireFriction.i
%include PxBatchQueryDesc.i

CSHARP_OBJECT_ARRAY(physx::PxVehicleWheels*, PxVehicleWheels)
CSHARP_ARRAYS(physx::PxVehicleWheelQueryResult, PxVehicleWheelQueryResult)
//CSHARP_BYREF_ARRAY2(physx::PxRaycastQueryResult, PxRaycastQueryResult)
CSHARP_ARRAYS(physx::PxVehicleConcurrentUpdateData, PxVehicleConcurrentUpdateData)
CSHARP_OBJECT_ARRAY(physx::PxVehicleWheelConcurrentUpdateData, PxVehicleWheelConcurrentUpdateData)

%apply physx::PxVehicleWheels* INPUT[] { PxVehicleWheels** vehicles }
//%apply physx::PxRaycastQueryResult BYREF[] { PxRaycastQueryResult* sceneQueryResults }
%apply bool INPUT[] { const bool* vehiclesToRaycast }
%typemap (cstype) PxRaycastQueryResult* "System.Runtime.InteropServices.HandleRef"
%typemap (imtype) PxRaycastQueryResult* "System.Runtime.InteropServices.HandleRef"
%typemap (csin) PxRaycastQueryResult* "$csinput" // Don't use the getCPtr() for HandleRefs
%typemap (csout, excode=SWIGEXCODE) PxRaycastQueryResult* {
    var ret = $imcall;$excode
    return ret;
}
void PxVehicleSuspensionRaycasts
		(PxBatchQuery* batchQuery,
		 const PxU32 nbVehicles, PxVehicleWheels** vehicles,
		 const PxU32 nbSceneQueryResults, PxRaycastQueryResult* sceneQueryResults,
		 const bool* vehiclesToRaycast = NULL);

struct PxWheelQueryResult
{
    PxWheelQueryResult()
    {
        PxMemZero(this, sizeof(PxWheelQueryResult));
        isInAir=true;
        tireSurfaceType = PxU32(PxVehicleDrivableSurfaceType::eSURFACE_TYPE_UNKNOWN);
        localPose = PxTransform(PxIdentity);
    }

    /**
    \brief Start point of suspension line raycast/sweep used in the raycast/sweep completed immediately before PxVehicleUpdates.
    \note If no raycast/sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then (0,0,0) is stored.
    @see PxVehicleSuspensionRaycasts, PxVehicleSuspensionRaycasts
    */
    PxVec3 suspLineStart;

    /**
    \brief Directions of suspension line raycast/sweep used in the raycast/sweep completed immediately before PxVehicleUpdates.
    \note If no raycast/sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then (0,0,0) is stored.
    @see PxVehicleSuspensionRaycasts, PxVehicleSuspensionRaycasts
    */
    PxVec3 suspLineDir;

    /**
    \brief Lengths of suspension line raycast/sweep used in raycast/sweep completed immediately before PxVehicleUpdates.
    \note If no raycast/sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then 0 is stored.
    @see PxVehicleSuspensionRaycasts, PxVehicleSuspensionRaycasts
    */
    PxReal suspLineLength;

    /**
    \brief If suspension travel limits forbid the wheel from touching the drivable surface then isInAir is true.
    \note If the wheel can be placed on the contact plane of the most recent suspension line raycast/sweep then isInAir is false.
    \note If #PxVehicleWheelsSimFlag::eLIMIT_SUSPENSION_EXPANSION_VELOCITY is set, then isInAir will also be true if the suspension
    force is not large enough to expand to the target length in the given simulation time step.
    \note If no raycast/sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then isInAir
    is computed using the contact plane that was hit by the most recent suspension line raycast/sweep.
    */
    bool isInAir;

    /**
    \brief PxActor instance of the driving surface under the corresponding vehicle wheel.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then tireContactActor is NULL.
    \note If no raycast/sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then NULL is stored.
    */
    PxActor* tireContactActor;

    /**
    \brief PxShape instance of the driving surface under the corresponding vehicle wheel.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then tireContactShape is NULL.
    \note If no raycast/sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then NULL is stored.
    */
    PxShape* tireContactShape;

    /**
    \brief PxMaterial instance of the driving surface under the corresponding vehicle wheel.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then tireSurfaceMaterial is NULL.
    \note If no raycast/sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then NULL is stored.
    */	
    const PxMaterial* tireSurfaceMaterial;

    /**
    \brief Surface type integer that corresponds to the mapping between tireSurfaceMaterial and integer as
    described in PxVehicleDrivableSurfaceToTireFrictionPairs.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then tireSurfaceType is 
    PxVehicleDrivableSurfaceType::eSURFACE_TYPE_UNKNOWN.
    \note If no raycast/sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then 
    PxVehicleDrivableSurfaceType::eSURFACE_TYPE_UNKNOWN is stored.
    @see PxVehicleDrivableSurfaceToTireFrictionPairs
    */	
    PxU32 tireSurfaceType;

    /**
    \brief Point on the drivable surface hit by the most recent suspension raycast or sweep.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then the contact point is (0,0,0).
    \note If no raycast or sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then (0,0,0) is stored.
    */
    PxVec3 tireContactPoint;

    /**
    \brief Normal on the drivable surface at the hit point of the most recent suspension raycast or sweep.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then the contact normal is (0,0,0).
    \note If no raycast or sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then (0,0,0) is stored.
    */
    PxVec3 tireContactNormal;

    /**
    \brief Friction experienced by the tire for the combination of tire type and surface type after accounting 
    for the friction vs slip graph.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then the tire friction is 0.
    \note If no raycast or sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then the 
    stored tire friction is the value computed in PxVehicleUpdates that immediately followed the last raycast or sweep.
    @see PxVehicleDrivableSurfaceToTireFrictionPairs, PxVehicleTireData
    */	
    PxReal tireFriction;

    /**
    \brief Compression of the suspension spring.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then the jounce is -PxVehicleSuspensionData.mMaxDroop
    The jounce can never exceed PxVehicleSuspensionData.mMaxCompression. Positive values result when the suspension is compressed from 
    the rest position, while negative values mean the suspension is elongated from the rest position.
    \note If no raycast or sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then the 
    suspension compression is computed using the contact plane that was hit by the most recent suspension line raycast or sweep.
    */	
    PxReal suspJounce;

    /**
    \brief Magnitude of force applied by the suspension spring along the direction of suspension travel.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then the force is 0
    \note If no raycast or sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then the 
    suspension spring force is computed using the contact plane that was hit by the most recent suspension line raycast or sweep.
    @see PxVehicleWheelsSimData::getSuspTravelDirection
    */
    PxReal suspSpringForce;

    /**
    \brief Forward direction of the wheel/tire accounting for steer/toe/camber angle projected on to the contact plane of the drivable surface.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then tireLongitudinalDir is (0,0,0)
    \note If no raycast or sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then the 
    tire longitudinal direction is computed using the contact plane that was hit by the most recent suspension line raycast or sweep.
    */
    PxVec3 tireLongitudinalDir;

    /**
    \brief Lateral direction of the wheel/tire accounting for steer/toe/camber angle projected on to the contact plan of the drivable surface.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then tireLateralDir is (0,0,0)
    \note If no raycast or sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then the 
    tire lateral direction is computed using the contact plane that was hit by the most recent suspension line raycast or sweep.
    */
    PxVec3 tireLateralDir;

    /**
    \brief Longitudinal slip of the tire.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then longitudinalSlip is 0.0
    \note The longitudinal slip is approximately (w*r - vz) / PxAbs(vz) where w is the angular speed of the wheel, r is the radius of the wheel, and 
    vz component of rigid body velocity computed at the wheel base along the longitudinal direction of the tire.
    \note If no raycast or sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then the 
    tire longitudinal slip is computed using the contact plane that was hit by the most recent suspension line raycast or sweep.
    */	
    PxReal longitudinalSlip;

    /**
    \brief Lateral slip of the tire.
    \note If suspension travel limits forbid the wheel from touching the drivable surface then lateralSlip is 0.0
    \note The lateral slip angle is approximately PxAtan(vx / PxAbs(vz)) where vx and vz are the components of rigid body velocity at the wheel base 
    along the wheel's lateral and longitudinal directions, respectively.
    \note If no raycast or sweep for the corresponding suspension was performed immediately prior to PxVehicleUpdates then the 
    tire lateral slip is computed using the contact plane that was hit by the most recent suspension line raycast or sweep.
    */	
    PxReal lateralSlip;

    /**
    \brief Steer angle of the wheel about the "up" vector accounting for input steer and toe and, if applicable, Ackermann steer correction.
    @see PxVehicleWheelData::mToeAngle
    */	
    PxReal steerAngle;

    /**
    \brief Local pose of the wheel.
    */
    PxTransform localPose;
};

struct PxVehicleWheelQueryResult
{
    /**
    \brief Pointer to an PxWheelQueryResult buffer of length nbWheelQueryResults
    The wheelQueryResults buffer must persist until the end of PxVehicleUpdates
    A NULL pointer is permitted.
    The wheelQueryResults buffer is left unmodified in PxVehicleUpdates for vehicles with sleeping rigid bodies 
    whose control inputs indicate they should remain inert.
    @see PxVehicleUpdates
    */
    PxWheelQueryResult* wheelQueryResults;

    /**
    \brief The length of the wheelQueryResults buffer.  This value corresponds to the 
    number of wheels in the associated vehicle in PxVehicleUpdates.
    */
    PxU32 nbWheelQueryResults;
};

struct PxVehicleWheelConcurrentUpdateData
{
    PxVehicleWheelConcurrentUpdateData()
        : localPose(PxTransform(PxIdentity)),
            hitActor(NULL),
            hitActorForce(PxVec3(0,0,0)),
            hitActorForcePosition(PxVec3(0,0,0))
    {
    }

private:

    PxTransform localPose;
    PxRigidDynamic* hitActor;
    PxVec3 hitActorForce;
    PxVec3 hitActorForcePosition;
};

struct PxVehicleConcurrentUpdateData
{
    PxVehicleConcurrentUpdateData()
        : concurrentWheelUpdates(NULL),
            nbConcurrentWheelUpdates(0),
            linearMomentumChange(PxVec3(0,0,0)),
            angularMomentumChange(PxVec3(0,0,0)),
            staySleeping(false),
            wakeup(false)
    {
    }

    /**
    \brief Pointer to an PxVehicleWheelConcurrentUpdate buffer of length nbConcurrentWheelUpdates
    The concurrentWheelUpdates buffer must persist until the end of PxVehiclePostUpdates
    A NULL pointer is not permitted.
    @see PxVehicleUpdates, PxVehiclePostUpdates
    */
    // TODO: Couldn't get the array to work yet
    //%apply physx::PxVehicleWheelConcurrentUpdateData OUTPUT[] { PxVehicleWheelConcurrentUpdateData* concurrentWheelUpdates }
    PxVehicleWheelConcurrentUpdateData* concurrentWheelUpdates;

    /**
    \brief The length of the concurrentWheelUpdates buffer.  This value corresponds to the 
    number of wheels in the associated vehicle passed to PxVehicleUpdates.
    */
    PxU32 nbConcurrentWheelUpdates;

private:
    PxVec3 linearMomentumChange;
    PxVec3 angularMomentumChange;
    bool staySleeping;
    bool wakeup;
};

%apply physx::PxVehicleWheels* INPUT[] { PxVehicleWheels** vehicles }
//%apply physx::PxVehicleWheelQueryResult OUTPUT[] { PxVehicleWheelQueryResult* vehicleWheelQueryResults }
%typemap (cstype) PxVehicleWheelQueryResult* "System.Runtime.InteropServices.HandleRef"
%typemap (imtype) PxVehicleWheelQueryResult* "System.Runtime.InteropServices.HandleRef"
%typemap (csin) PxVehicleWheelQueryResult* "$csinput" // pass the HandleRef directly without CPtr
%typemap (csout, excode=SWIGEXCODE) PxVehicleWheelQueryResult* {
    var ret = $imcall;$excode
    return ret;
}

%apply physx::PxVehicleConcurrentUpdateData OUTPUT[] { PxVehicleConcurrentUpdateData* vehicleConcurrentUpdates }
void PxVehicleUpdates(
    const PxReal timestep, const PxVec3& gravity,
    const PxVehicleDrivableSurfaceToTireFrictionPairs& vehicleDrivableSurfaceToTireFrictionPairs,
    const PxU32 nbVehicles, PxVehicleWheels** vehicles, PxVehicleWheelQueryResult* vehicleWheelQueryResults,
    PxVehicleConcurrentUpdateData* vehicleConcurrentUpdates = NULL);

%include PxVehicleUtilTelemetry.i

void PxVehicleUpdateSingleVehicleAndStoreTelemetryData
		(const PxReal timestep, const PxVec3& gravity, 
		 const PxVehicleDrivableSurfaceToTireFrictionPairs& vehicleDrivableSurfaceToTireFrictionPairs, 
		 PxVehicleWheels* focusVehicle, PxVehicleWheelQueryResult* vehicleWheelQueryResults, 
		 PxVehicleTelemetryData& telemetryData,
		 PxVehicleConcurrentUpdateData* vehicleConcurrentUpdates = NULL);

void PxVehicleShiftOrigin(const PxVec3& shift, const PxU32 nbVehicles, PxVehicleWheels** vehicles);
