// TODO: Can we hand-generate these limits?
/*const eMAX_NB_SAMPLES=256
const eMAX_NB_TITLE_CHARS=256
const eMAX_NB_CHANNELS=12*/

class PxVehicleGraphDesc
{
	PxVehicleGraphDesc();
	PxReal mPosX;
    PxReal mPosY;
    PxReal mSizeX;
    PxReal mSizeY;
    PxVec3 mBackgroundColor;
    PxReal mAlpha;
};

struct PxVehicleGraphChannelDesc
{
public:
	PxVehicleGraphChannelDesc();
	PxReal mMinY;
	PxReal mMaxY;
	PxReal mMidY;
	PxVec3 mColorLow;
	PxVec3 mColorHigh;
	char* mTitle;
};

SIMPLIFY_ENUM(PxVehicleWheelGraphChannel,
JOUNCE,
SUSPFORCE,
TIRELOAD,
NORMALIZED_TIRELOAD,
WHEEL_OMEGA,
TIRE_FRICTION,
TIRE_LONG_SLIP,
NORM_TIRE_LONG_FORCE,
TIRE_LAT_SLIP,
NORM_TIRE_LAT_FORCE,
NORM_TIRE_ALIGNING_MOMENT,
MAX_NB_WHEEL_CHANNELS
)

SIMPLIFY_ENUM(PxVehicleDriveGraphChannel,
ENGINE_REVS=0,
ENGINE_DRIVE_TORQUE,
CLUTCH_SLIP,
ACCEL_CONTROL,					//TANK_ACCEL
BRAKE_CONTROL,					//TANK_BRAKE_LEFT
HANDBRAKE_CONTROL,				//TANK_BRAKE_RIGHT
STEER_LEFT_CONTROL,			//TANK_THRUST_LEFT
STEER_RIGHT_CONTROL,			//TANK_THRUST_RIGHT
GEAR_RATIO,
MAX_NB_DRIVE_CHANNELS
)
SIMPLIFY_ENUM(PxVehicleGraphType, WHEEL, DRIVE)

class PxVehicleGraph
{
public:
	void setup(const PxVehicleGraphDesc& desc, const PxVehicleGraphType::Enum graphType);
	void clearRecordedChannelData();
    const PxVec3& getBackgroundColor() const;
	PxReal getBackgroundAlpha() const;
	void getBackgroundCoords(PxReal& xMin, PxReal& yMin, PxReal& xMax, PxReal& yMax) const;
	void computeGraphChannel(const PxU32 channel, PxReal* xy, PxVec3* colors, char* title) const;
	PxF32 getLatestValue(const PxU32 channel) const;
    %apply float OUTPUT[] { PxReal* values }
	void getRawData(const PxU32 channel, PxReal* values) const;
    private:
      ~PxVehicleGraph();
};

class PxVehicleTelemetryData
{
public:
	static PxVehicleTelemetryData* allocate(const PxU32 nbWheels);
	void free();
    %apply float INPUT[] { PxReal* wheelGraphPosX }
    %apply float INPUT[] { PxReal* wheelGraphPosY }
	void setup
		(const PxReal graphSizeX, const PxReal graphSizeY,
		 const PxReal engineGraphPosX, const PxReal engineGraphPosY,
		 const PxReal* const wheelGraphPosX, const PxReal* const wheelGraphPosY,
		 const PxVec3& backGroundColor, const PxVec3& lineColorHigh, const PxVec3& lineColorLow);
	void clear();
	const PxVehicleGraph& getEngineGraph() const;
	PxU32 getNbWheelGraphs() const;
	const PxVehicleGraph& getWheelGraph(const PxU32 k) const;
	const PxVec3* getTireforceAppPoints() const;
	const PxVec3* getSuspforceAppPoints() const;
private:
    ~PxVehicleTelemetryData();
};