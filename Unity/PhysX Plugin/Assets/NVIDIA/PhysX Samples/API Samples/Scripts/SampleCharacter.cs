using System;
using NVIDIA.PhysX;
using NVIDIA.PhysX.UnityExtensions;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Timers;
using UnityEditor.PackageManager;
using UnityEngine;
using Debug = UnityEngine.Debug;

public class SampleCharacter : SampleBase
{
    const uint COLLISION_FLAG_GROUND			=	1 << 0;
    const uint COLLISION_FLAG_WHEEL			=	1 << 1;
    const uint COLLISION_FLAG_CHASSIS			=	1 << 2;
    const uint COLLISION_FLAG_OBSTACLE			=	1 << 3;
    const uint COLLISION_FLAG_DRIVABLE_OBSTACLE  =	1 << 4;

    private const uint COLLISION_FLAG_GROUND_AGAINST =
        COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE;

    private const uint COLLISION_FLAG_WHEEL_AGAINST =
        COLLISION_FLAG_WHEEL | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE;

    private const uint COLLISION_FLAG_CHASSIS_AGAINST = COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL |
                                                        COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE |
                                                        COLLISION_FLAG_DRIVABLE_OBSTACLE;

    private const uint COLLISION_FLAG_OBSTACLE_AGAINST = COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL |
                                                         COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE |
                                                         COLLISION_FLAG_DRIVABLE_OBSTACLE;

    private const uint COLLISION_FLAG_DRIVABLE_OBSTACLE_AGAINST = COLLISION_FLAG_GROUND | COLLISION_FLAG_CHASSIS |
                                                                  COLLISION_FLAG_OBSTACLE |
                                                                  COLLISION_FLAG_DRIVABLE_OBSTACLE;

    private PxRigidDynamic m_vehicleActor;
    private PxVehicleDrive4W m_vehicle;
    private VehicleSceneQueryData gVehicleSceneQueryData;
    private PxBatchQuery gBatchQuery;
    private PxVehicleDrivableSurfaceToTireFrictionPairs gFrictionPairs;

    #region Messages

    private void FixedUpdate()
    {
        if (m_scene != null)
        {
            if (m_throwBall) ThrowBall();
            StepVehicle();
            m_scene.simulate(Time.fixedDeltaTime);
            m_scene.fetchResults(true);
        }
    }

    private void Update()
    {
        UpdateCamera();
        UpdateInput();
        UpdatePicker(m_scene);
    }

    #endregion

    #region Protected

    protected override void CreateSample()
    {
        base.CreateSample();
        CreateScene();
    }

    protected override void DestroySample()
    {
        DestroyScene();
        base.DestroySample();
    }

    #endregion

    #region Private

    void CreateScene()
    {
        // Create CPU dispatcher
        m_cpuDispatcher = PxCpuDispatcher.createDefault((uint)SystemInfo.processorCount);
        
        // Those could be set later so we don't need to hardcode them
        Native.PxVehicleSetBasisVectors(new PxVec3(0, 1, 0), new PxVec3(0, 0, 1));
        Native.PxVehicleSetUpdateMode(PxVehicleUpdateMode.VELOCITY_CHANGE);

        // Create PxScene
        var sceneDesc = new PxSceneDesc(physics.getTolerancesScale());
        sceneDesc.cpuDispatcher = m_cpuDispatcher;
        sceneDesc.filterShader = PxDefaultSimulationFilterShader.function;
        sceneDesc.gravity = new PxVec3(0, -9.8f, 0);
        sceneDesc.broadPhaseType = PxBroadPhaseType.ABP;
        m_scene = physics.createScene(sceneDesc);
        sceneDesc.destroy();

        // Set PVD flags
        var pvdClient = m_scene.getScenePvdClient();
        if (pvdClient != null)
            pvdClient.setScenePvdFlags(PxPvdSceneFlag.TRANSMIT_CONTACTS | PxPvdSceneFlag.TRANSMIT_CONSTRAINTS | PxPvdSceneFlag.TRANSMIT_SCENEQUERIES);

        m_physicsMaterial = physics.createMaterial(0.5f, 0.5f, 0.05f);
        
        CreateGround();
        CreateObjects();
        CreateCharacter();
    }

    void DestroyScene()
    {
        foreach (var a in m_dynamicActors) a?.release();
        m_dynamicActors.Clear();
        Destroy(m_boxMesh);
        Destroy(m_ballMesh);
        Destroy(m_activeBoxMaterial);
        m_activeBoxMaterial = null;
        Destroy(m_inactiveBoxMaterial);
        m_inactiveBoxMaterial = null;
        Destroy(m_activeBallMaterial);
        m_activeBallMaterial = null;
        Destroy(m_inactiveBallMaterial);
        m_inactiveBallMaterial = null;
        m_boxMesh = null;
        m_groundActor?.release();
        m_groundActor = null;
        Destroy(m_groundMesh);
        m_groundMesh = null;
        Destroy(m_groundMaterial);
        m_groundMaterial = null;
        m_scene?.release();
        m_scene = null;
        m_physicsMaterial?.release();
        m_physicsMaterial = null;
        m_cpuDispatcher?.release();
        m_cpuDispatcher = null;
    }

    void StepVehicle()
    {
        var sw = Stopwatch.StartNew();
        sw.Start();
        if (m_vehicle == null)
        {
            Debug.Log("Vehicle not initialized yet");
            return;
        }
        
        var gravity = new PxVec3(0f, -9.81f, 0f);
        PxVehicleWheels[] vehicles = { m_vehicle };
        var raycastResults = gVehicleSceneQueryData.getRaycastQueryResultBuffer(0);
        var raycastResultSize = gVehicleSceneQueryData.getQueryResultBufferSize();
        //var batchQuery = VehicleSceneQueryData.setUpBatchedSceneQuery(0, gVehicleSceneQueryData, m_scene);
        Native.PxVehicleSuspensionRaycasts(gBatchQuery, 1, vehicles, raycastResultSize,
            new HandleRef(null, raycastResults));
        
        
        Native.PxVehicleUpdates(Time.fixedDeltaTime, gravity, gFrictionPairs, 1, vehicles, new HandleRef(null, IntPtr.Zero));

        var pos = m_vehicle.getRigidDynamicActor().getGlobalPose().p;
        sw.Stop();
        
        Debug.Log($"{pos.x}, {pos.y}, {pos.z} at {m_vehicle.mDriveDynData.getEngineRotationSpeed() * (30 / Mathf.PI)} RPM in Gear {m_vehicle.mDriveDynData.getCurrentGear()} [Sim Time: {sw.Elapsed.TotalMilliseconds}]");
    }
    
    void CreateCharacter()
    {
        var desc = new PxCapsuleControllerDesc();
        desc.height = 2f;
        desc.radius = 0.3f;
        desc.maxJumpHeight = 2f;

        var mgr = Native.PxCreateControllerManager(m_scene);
        var ctrl = mgr.createController(desc);
        ctrl.setPosition(new PxExtendedVec3(0d, 0d, 0d));
        ctrl.move(new PxVec3(0f, 1f, 0f), 1f, Timer.fixedDeltaTime, new PxControllerFilters(new PxFilterData()));
    }

    private static PxQueryHitType WheelSceneQueryPreFilterBlocking(PxFilterData queryfilterdata, PxFilterData objectfilterdata, IntPtr constantblock, uint constantblocksize, PxHitFlag hitflags)
    {
        // QueryFilterData is the suspension query
        // ObjectFilterData is the shape potentially hit by the query.
        return ((0 == (objectfilterdata.word3 & SnippetVehicleSceneQuery.DRIVABLE_SURFACE)) ? PxQueryHitType.NONE : PxQueryHitType.BLOCK);
    }

    private PxRigidDynamic SetupActor(PxCooking cooking, float wheelWidth, float wheelRadius, uint numWheels,
        PxVehicleChassisData chassis, PxVec3 chassisDims, PxMaterial wheelMaterial, PxMaterial chassisMaterial)
    {
        //Construct a convex mesh for a cylindrical wheel.
        PxConvexMesh wheelMesh = CreateWheelMesh(wheelWidth, wheelRadius, physics, cooking);
        //Assume all wheels are identical for simplicity.
        PxConvexMesh[] wheelConvexMeshes = new PxConvexMesh[numWheels];
        PxMaterial[] wheelMaterials = new PxMaterial[numWheels];

        //Set the meshes and materials for the driven wheels.
        for(int i = (int)PxVehicleDrive4WWheelOrder.Enum.FRONT_LEFT; i <= (int)PxVehicleDrive4WWheelOrder.Enum.REAR_RIGHT; i++)
        {
            wheelConvexMeshes[i] = wheelMesh;
            wheelMaterials[i] = wheelMaterial;
        }
        //Set the meshes and materials for the non-driven wheels
        for(int i = (int)PxVehicleDrive4WWheelOrder.Enum.REAR_RIGHT + 1; i < numWheels; i++)
        {
            wheelConvexMeshes[i] = wheelMesh;
            wheelMaterials[i] = wheelMaterial;
        }

        //Chassis just has a single convex shape for simplicity.
        PxConvexMesh chassisConvexMesh = createChassisMesh(chassisDims, physics, cooking);
        PxConvexMesh[] chassisConvexMeshes = new[] { chassisConvexMesh };
        PxMaterial[] chassisMaterials = new[] { chassisMaterial };

        //Rigid body data.
        PxVehicleChassisData rigidBodyData = new PxVehicleChassisData
        {
            mMOI = chassis.mMOI,
            mMass = chassis.mMass,
            mCMOffset = chassis.mCMOffset
        };

        var chassisSimFilterData = new PxFilterData(COLLISION_FLAG_CHASSIS, COLLISION_FLAG_CHASSIS_AGAINST, 0, 0);
        var wheelSimFilterData = new PxFilterData(COLLISION_FLAG_WHEEL, COLLISION_FLAG_WHEEL_AGAINST, 0, 0);

        return createVehicleActor (rigidBodyData, wheelMaterials, wheelConvexMeshes, numWheels, wheelSimFilterData,
            chassisMaterials, chassisConvexMeshes, 1, chassisSimFilterData);
    }

    private PxConvexMesh CreateWheelMesh(float width, float radius, PxPhysics pxPhysics, PxCooking cooking)
    {
        var points = new PxVec3[32];
        for(int i = 0; i < 16; i++)
        {
            var cosTheta = Math.Cos(i* Math.PI * 2.0f/16.0f);
            var sinTheta = Math.Sin(i * Math.PI * 2.0f/16.0f);
            var y = radius * (float)cosTheta;
            var z = radius * (float)sinTheta;
            points[2 * i + 0] = new PxVec3(-width/2.0f, y, z);
            points[2 * i + 1] = new PxVec3(+width/2.0f, y, z);
        }

        return createConvexMesh(points, physics, cooking);
    }
    
    PxConvexMesh createChassisMesh(PxVec3 dims, PxPhysics physics, PxCooking cooking)
    {
        var x = dims.x*0.5f;
        var y = dims.y*0.5f;
        var z = dims.z*0.5f;
        var verts =
        new[] {
            new PxVec3(x,y,-z), 
            new PxVec3(x,y,z),
            new PxVec3(x,-y,z),
            new PxVec3(x,-y,-z),
            new PxVec3(-x,y,-z), 
            new PxVec3(-x,y,z),
            new PxVec3(-x,-y,z),
            new PxVec3(-x,-y,-z)
        };

        return createConvexMesh(verts, physics, cooking);
    }

    
    static PxConvexMesh createConvexMesh(PxVec3[] verts, PxPhysics physics, PxCooking cooking)
    {
        // Managed memory should be pinned before passing it to native function
        var pinVerts = GCHandle.Alloc(verts, GCHandleType.Pinned);
       
        // Create descriptor for convex mesh
        var convexDesc = new PxConvexMeshDesc
        {
            points =
            {
                count = (uint)verts.Length,
                stride = sizeof(float) * 3,
                data = Marshal.UnsafeAddrOfPinnedArrayElement(verts, 0)
            },
            flags = PxConvexFlag.COMPUTE_CONVEX
        };

        // Create PxConvexMesh
        var convexMesh = cooking.createConvexMesh(convexDesc, physics.getPhysicsInsertionCallback(), out var condition);
        if (condition != PxConvexMeshCookingResult.SUCCESS)
        {
            Debug.LogError("PhysX Cooking: Failed to create convex mesh.");
        }

        convexDesc.destroy(); // This destroys native instance of PxConvexMeshDesc. If not called explicitly, GC will call it when finalizing
        pinVerts.Free(); // Unpin managed memory
        
        return convexMesh;
    }

    private PxVec3[] computeWheelCenterActorOffset4W(float wheelFrontZ, float wheelRearZ, PxVec3 chassisDims, float wheelWidth, float wheelRadius, uint numWheels)
    {
        var wheelCentreOffsets = new PxVec3[numWheels];
        
        //chassisDims.z is the distance from the rear of the chassis to the front of the chassis.
        //The front has z = 0.5*chassisDims.z and the rear has z = -0.5*chassisDims.z.
        //Compute a position for the front wheel and the rear wheel along the z-axis.
        //Compute the separation between each wheel along the z-axis.
        float numLeftWheels = numWheels/2.0f;
        float deltaZ = (wheelFrontZ - wheelRearZ)/(numLeftWheels-1.0f);
        //Set the outside of the left and right wheels to be flush with the chassis.
        //Set the top of the wheel to be just touching the underside of the chassis.
        //Begin by setting the rear-left/rear-right/front-left,front-right wheels.
        wheelCentreOffsets[(int)PxVehicleDrive4WWheelOrder.Enum.REAR_LEFT] = new PxVec3((-chassisDims.x + wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + 0*deltaZ*0.5f);
        wheelCentreOffsets[(int)PxVehicleDrive4WWheelOrder.Enum.REAR_RIGHT] = new PxVec3((+chassisDims.x - wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + 0*deltaZ*0.5f);
        wheelCentreOffsets[(int)PxVehicleDrive4WWheelOrder.Enum.FRONT_LEFT] = new PxVec3((-chassisDims.x + wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + (numLeftWheels-1)*deltaZ);
        wheelCentreOffsets[(int)PxVehicleDrive4WWheelOrder.Enum.FRONT_RIGHT] = new PxVec3((+chassisDims.x - wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + (numLeftWheels-1)*deltaZ);
        //Set the remaining wheels.
        for(int i = 2, wheelCount = 4; i < numWheels-2; i+=2, wheelCount+=2)
        {
            wheelCentreOffsets[wheelCount + 0] = new PxVec3((-chassisDims.x + wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + i*deltaZ*0.5f);
            wheelCentreOffsets[wheelCount + 1] = new PxVec3((+chassisDims.x - wheelWidth)*0.5f, -(chassisDims.y/2 + wheelRadius), wheelRearZ + i*deltaZ*0.5f);
        }

        return wheelCentreOffsets;
    }

    private PxVehicleDriveSimData4W setupDriveSimData(PxVehicleWheelsSimData wheelsSimData)
    {
        var driveSimData = new PxVehicleDriveSimData4W();
        var diff = new PxVehicleDifferential4WData
        {
            mType = PxVehicleDifferential4WData.Enum.DIFF_TYPE_LS_4WD
        };
        driveSimData.setDiffData(diff);

        //Engine
        var engine = new PxVehicleEngineData
        {
            mPeakTorque = 500.0f,
            mMaxOmega = 600.0f //approx 6000 rpm
        };
        driveSimData.setEngineData(engine);

        //Gears
        var gears = new PxVehicleGearsData
        {
            mSwitchTime = 0.5f
        };
        driveSimData.setGearsData(gears);

        //Clutch
        var clutch = new PxVehicleClutchData
        {
            mStrength = 10.0f
        };
        driveSimData.setClutchData(clutch);

        //Ackermann steer accuracy
        var ackermann = new PxVehicleAckermannGeometryData
        {
            mAccuracy = 1.0f,
            mAxleSeparation = wheelsSimData.getWheelCentreOffset((uint)PxVehicleDrive4WWheelOrder.Enum.FRONT_LEFT).z-
                              wheelsSimData.getWheelCentreOffset((uint)PxVehicleDrive4WWheelOrder.Enum.REAR_LEFT).z,
            mFrontWidth = wheelsSimData.getWheelCentreOffset((uint)PxVehicleDrive4WWheelOrder.Enum.FRONT_RIGHT).x-
                          wheelsSimData.getWheelCentreOffset((uint)PxVehicleDrive4WWheelOrder.Enum.FRONT_LEFT).x,
            mRearWidth = wheelsSimData.getWheelCentreOffset((uint)PxVehicleDrive4WWheelOrder.Enum.REAR_RIGHT).x-
                         wheelsSimData.getWheelCentreOffset((uint)PxVehicleDrive4WWheelOrder.Enum.REAR_LEFT).x
        };
        driveSimData.setAckermannGeometryData(ackermann);

        return driveSimData;
    }

    private PxVehicleWheelsSimData setupWheelsSimulationData(float wheelMass, float wheelMOI, float wheelRadius, float wheelWidth, PxVec3[] wheelCenterActorOffsets, PxVec3 chassisCMOffset /* 0, 0, 0 */, float chassisMass)
    {
        var wheelsSimData = PxVehicleWheelsSimData.allocate(4);
        var wheels = new PxVehicleWheelData[4];
        for (int i = 0; i < 4; i++)
        {
            wheels[i] = new PxVehicleWheelData
            {
                mMass = wheelMass,
                mMOI = wheelMOI,
                mRadius = wheelRadius,
                mWidth = wheelWidth
            };
        }

        wheels[2].mMaxHandBrakeTorque = 4000f;
        wheels[3].mMaxHandBrakeTorque = 4000f;
        wheels[0].mMaxSteer = Mathf.PI * 0.3333f;
        wheels[1].mMaxSteer = Mathf.PI * 0.3333f;

        var tires = new PxVehicleTireData[4];
        for (int i = 0; i < 4; i++)
        {
            tires[i] = new PxVehicleTireData
            {
                mType = 0 // TIRE_TYPE_NORMAL
            };
        }
        const int numWheels = 4;
        var suspensions = new PxVehicleSuspensionData[4];
        float[] suspSprungMasses = new float[4];
        const uint upDirection = 1u;
        Native.PxVehicleComputeSprungMasses(4, wheelCenterActorOffsets, 
            chassisCMOffset, chassisMass, upDirection, suspSprungMasses);
        
        for (int i = 0; i < 4; i++)
        {
            suspensions[i] = new PxVehicleSuspensionData()
            {
                mMaxCompression = 0.3f,
                mMaxDroop = 0.1f,
                mSpringStrength = 35000.0f,
                mSpringDamperRate = 4500f,
                mSprungMass = suspSprungMasses[i]
            };
        }
        
        //Set the camber angles.
        const float camberAngleAtRest = 0.0f;
        const float camberAngleAtMaxDroop = 0.01f;
        const float camberAngleAtMaxCompression = -0.01f;
        for (int i = 0; i < 4; i += 2)
        {
            suspensions[i + 0].mCamberAtRest =  camberAngleAtRest;
            suspensions[i + 1].mCamberAtRest =  -camberAngleAtRest;
            suspensions[i + 0].mCamberAtMaxDroop = camberAngleAtMaxDroop;
            suspensions[i + 1].mCamberAtMaxDroop = -camberAngleAtMaxDroop;
            suspensions[i + 0].mCamberAtMaxCompression = camberAngleAtMaxCompression;
            suspensions[i + 1].mCamberAtMaxCompression = -camberAngleAtMaxCompression;
        }
        
        //Set up the wheel geometry.
        PxVec3[] suspTravelDirections = new PxVec3[4];
        PxVec3[] wheelCentreCMOffsets = new PxVec3[4];
        PxVec3[] suspForceAppCMOffsets = new PxVec3[4];
        PxVec3[] tireForceAppCMOffsets = new PxVec3[4];
        {
            //Set the geometry data.
            for(int i = 0; i < numWheels; i++)
            {
                //Vertical suspension travel.
                suspTravelDirections[i] = new PxVec3(0,-1,0);
    
                //Wheel center offset is offset from rigid body center of mass.
                wheelCentreCMOffsets[i] =
                    wheelCenterActorOffsets[i] - chassisCMOffset;
    
                //Suspension force application point 0.3 metres below
                //rigid body center of mass.
                suspForceAppCMOffsets[i] =
                    new PxVec3(wheelCentreCMOffsets[i].x,-0.3f,wheelCentreCMOffsets[i].z);
    
                //Tire force application point 0.3 metres below
                //rigid body center of mass.
                tireForceAppCMOffsets[i] =
                    new PxVec3(wheelCentreCMOffsets[i].x,-0.3f,wheelCentreCMOffsets[i].z);
            }
        }
    
        //Set up the filter data of the raycast that will be issued by each suspension.
        var qryFilterData = SnippetVehicleSceneQuery.setupNonDrivableSurface();
        
        //Set the wheel, tire and suspension data.
        //Set the geometry data.
        //Set the query filter data
        for(uint i = 0; i < numWheels; i++)
        {
            wheelsSimData.setWheelData(i, wheels[i]);
            wheelsSimData.setTireData(i, tires[i]);
            wheelsSimData.setSuspensionData(i, suspensions[i]);
            wheelsSimData.setSuspTravelDirection(i, suspTravelDirections[i]);
            wheelsSimData.setWheelCentreOffset(i, wheelCentreCMOffsets[i]);
            wheelsSimData.setSuspForceAppPointOffset(i, suspForceAppCMOffsets[i]);
            wheelsSimData.setTireForceAppPointOffset(i, tireForceAppCMOffsets[i]);
            wheelsSimData.setSceneQueryFilterData(i, qryFilterData);
            wheelsSimData.setWheelShapeMapping(i, (int)i);
        }

        return wheelsSimData;
    }
    
    PxRigidDynamic createVehicleActor(PxVehicleChassisData chassisData, PxMaterial[] wheelMaterials, 
        PxConvexMesh[] wheelConvexMeshes, uint numWheels, PxFilterData wheelSimFilterData,
        PxMaterial[] chassisMaterials, PxConvexMesh[] chassisConvexMeshes, uint numChassisMeshes, 
        PxFilterData chassisSimFilterData)
    {
        //We need a rigid body actor for the vehicle.
        //Don't forget to add the actor to the scene after setting up the associated vehicle.
        PxRigidDynamic vehActor = physics.createRigidDynamic(new PxTransform(PxIDENTITY.PxIdentity));

        //Wheel and chassis query filter data.
        //Optional: cars don't drive on other cars.
        PxFilterData wheelQryFilterData = SnippetVehicleSceneQuery.setupNonDrivableSurface();
        PxFilterData chassisQryFilterData = SnippetVehicleSceneQuery.setupNonDrivableSurface();

        //Add all the wheel shapes to the actor.
        for(uint i = 0; i < numWheels; i++)
        {
            PxConvexMeshGeometry geom = new PxConvexMeshGeometry(wheelConvexMeshes[i]);
            PxShape wheelShape = PxRigidActorExt.createExclusiveShape(vehActor, geom, wheelMaterials[i]);
            wheelShape.setQueryFilterData(wheelQryFilterData);
            wheelShape.setSimulationFilterData(wheelSimFilterData);
            wheelShape.setLocalPose(new PxTransform(PxIDENTITY.PxIdentity));
        }

        //Add the chassis shapes to the actor.
        for(uint i = 0; i < numChassisMeshes; i++)
        {
            PxShape chassisShape = PxRigidActorExt.createExclusiveShape(vehActor, new PxConvexMeshGeometry(chassisConvexMeshes[i]), chassisMaterials[i]);
            chassisShape.setQueryFilterData(chassisQryFilterData);
            chassisShape.setSimulationFilterData(chassisSimFilterData);
            chassisShape.setLocalPose(new PxTransform(PxIDENTITY.PxIdentity));
        }

        vehActor.setMass(chassisData.mMass);
        vehActor.setMassSpaceInertiaTensor(chassisData.mMOI);
        vehActor.setCMassLocalPose(new PxTransform(chassisData.mCMOffset, new PxQuat(0, 0, 0, 1)));

        return vehActor;
    }

    void CreateGround()
    {
        // Create PxRigidStatic for ground
        m_groundActor = physics.createRigidStatic(new PxTransform(Quaternion.Euler(0, 0, 90.0f).ToPxQuat()));
        // Add plane shape
        var shape = m_groundActor.createExclusiveShape(new PxPlaneGeometry(), m_physicsMaterial);
        // Add ground actor to scene
        m_scene.addActor(m_groundActor);

        m_groundMesh = new Mesh();
        m_groundMesh.vertices = new[] { new Vector3(0, 1000, 1000), new Vector3(0, -1000, 1000), new Vector3(0, -1000, -1000), new Vector3(0, 1000, -1000) };
        m_groundMesh.normals = new[] { Vector3.right, Vector3.right, Vector3.right, Vector3.right };
        m_groundMesh.uv = new[] { Vector2.zero, Vector2.zero, Vector2.zero, Vector2.zero };
        m_groundMesh.triangles = new[] { 0, 1, 2, 0, 2, 3 };
        m_groundMesh.RecalculateBounds();
        m_groundMaterial = new Material(Shader.Find("Standard"));
        m_groundMaterial.color = COLOR3;

        var groundPlaneSimFilterData = new PxFilterData(COLLISION_FLAG_GROUND, COLLISION_FLAG_GROUND_AGAINST, 0, 0);
        shape.setSimulationFilterData(groundPlaneSimFilterData);
        shape.setQueryFilterData(SnippetVehicleSceneQuery.setupDrivableSurface());
        AddRenderActor(m_groundActor, m_groundMesh, m_groundMaterial);
    }

    void CreateObjects()
    {
        m_activeBoxMaterial = new Material(Shader.Find("Standard"));
        m_activeBoxMaterial.color = COLOR4;
        m_activeBoxMaterial.enableInstancing = true;
        m_inactiveBoxMaterial = new Material(Shader.Find("Standard"));
        m_inactiveBoxMaterial.color = COLOR3;
        m_inactiveBoxMaterial.enableInstancing = true;

        m_activeBallMaterial = new Material(Shader.Find("Standard"));
        m_activeBallMaterial.color = COLOR5;
        m_activeBallMaterial.enableInstancing = true;
        m_inactiveBallMaterial = new Material(Shader.Find("Standard"));
        m_inactiveBallMaterial.color = COLOR6;
        m_inactiveBallMaterial.enableInstancing = true;

        m_boxMesh = CreateBoxMesh(1, 1, 1);
        m_ballMesh = CreateSphereMesh(0.5f, 0.01f);

        for (int i = 0; i < 5; ++i)
            CreatePyramid(Vector3.forward * 2.0f * i, 20);
    }

    void CreatePyramid(Vector3 pos, int size)
    {
        return;
        for (int y = 0; y < size; ++y)
        {
            var start = pos - Vector3.right * 0.5f * (size - 1 - y) + Vector3.up * (y + 0.5f);
            for (int i = 0; i < size - y; ++i)
            {
                // Create PxRigidDynamic
                var body = physics.createRigidDynamic(new PxTransform((start + Vector3.right * i).ToPxVec3()));
                // Add box shape
                body.createExclusiveShape(new PxBoxGeometry(0.5f, 0.5f, 0.5f), m_physicsMaterial);
                // Compute rigid body mass and inertia from its shapes (default density is 1000)
                body.updateMassAndInertia();
                // Add body to scene
                m_scene.addActor(body);

                m_dynamicActors.Add(body);
                AddRenderActor(body, m_boxMesh, m_activeBoxMaterial, m_inactiveBoxMaterial);
            }
        }
    }

    void ThrowBall()
    {
        var pos = Camera.main.transform.position + Camera.main.transform.forward * 2 + Camera.main.transform.up * -2;
        var rot = Camera.main.transform.rotation;

        // Create PxRigidDynamic
        var body = physics.createRigidDynamic(new PxTransform(pos.ToPxVec3(), rot.ToPxQuat()));
        // Add sphere shape
        body.createExclusiveShape(new PxSphereGeometry(0.5f), m_physicsMaterial);
        // Set body mass and compute inertia
        body.setMassAndUpdateInertia(5000);
        // Set body velocity
        body.setLinearVelocity((Camera.main.transform.forward * 80).ToPxVec3());
        // Add body to scene
        m_scene.addActor(body);

        m_dynamicActors.Add(body);
        AddRenderActor(body, m_ballMesh, m_activeBallMaterial, m_inactiveBallMaterial);
    }

    void UpdateInput()
    {
        m_throwBall = Input.GetKey(KeyCode.Space);
    }

    void UpdateCamera()
    {
        const float MOTION_SPEED = 20.0f;
        const float ROTATION_SPEED = 0.3f;
        float forward = 0, right = 0;
        if (Input.GetKey(KeyCode.W)) forward += 1;
        if (Input.GetKey(KeyCode.S)) forward -= 1;
        if (Input.GetKey(KeyCode.D)) right += 1;
        if (Input.GetKey(KeyCode.A)) right -= 1;
        float dt = Time.deltaTime;
        var cameraTransform = Camera.main.transform;
        cameraTransform.Translate(Vector3.forward * forward * MOTION_SPEED * dt + Vector3.right * right * MOTION_SPEED * dt, Space.Self);

        if (Input.GetMouseButtonDown(1))
        {
            m_mousePosition = Input.mousePosition;
        }
        if (Input.GetMouseButton(1))
        {
            var mouseDelta = Input.mousePosition - m_mousePosition;
            cameraTransform.Rotate(Vector3.up, mouseDelta.x * ROTATION_SPEED, Space.World);
            cameraTransform.Rotate(Vector3.left, mouseDelta.y * ROTATION_SPEED, Space.Self);
            m_mousePosition = Input.mousePosition;
        }
    }

    PxDefaultCpuDispatcher m_cpuDispatcher;
    PxScene m_scene;
    PxMaterial m_physicsMaterial;
    PxRigidStatic m_groundActor;
    Mesh m_groundMesh, m_boxMesh, m_ballMesh;
    Material m_groundMaterial, m_activeBoxMaterial, m_inactiveBoxMaterial, m_activeBallMaterial, m_inactiveBallMaterial;
    List<PxRigidDynamic> m_dynamicActors = new List<PxRigidDynamic>();
    Vector3 m_mousePosition;
    bool m_throwBall = false;

    #endregion
}
