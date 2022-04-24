struct PxVehicleKeySmoothingData
{
    public:
        PxReal mRiseRates[PxVehicleDriveDynData::eMAX_NB_ANALOG_INPUTS];
        PxReal mFallRates[PxVehicleDriveDynData::eMAX_NB_ANALOG_INPUTS];
};

struct PxVehiclePadSmoothingData
{
    public:
        PxReal mRiseRates[PxVehicleDriveDynData::eMAX_NB_ANALOG_INPUTS];
        PxReal mFallRates[PxVehicleDriveDynData::eMAX_NB_ANALOG_INPUTS];
};

class PxVehicleDrive4WRawInputData
{
    public:
        PxVehicleDrive4WRawInputData();
        virtual ~PxVehicleDrive4WRawInputData();
        void setDigitalAccel(const bool accelKeyPressed);
        void setDigitalBrake(const bool brakeKeyPressed);
        void setDigitalHandbrake(const bool handbrakeKeyPressed);
        void setDigitalSteerLeft(const bool steerLeftKeyPressed);
        void setDigitalSteerRight(const bool steerRightKeyPressed);
        bool getDigitalAccel() const;
        bool getDigitalBrake() const;
        bool getDigitalHandbrake() const;
        bool getDigitalSteerLeft() const;
        bool getDigitalSteerRight() const;
        void setAnalogAccel(const PxReal accel);
        void setAnalogBrake(const PxReal brake);
        void setAnalogHandbrake(const PxReal handbrake);
        void setAnalogSteer(const PxReal steer);
        PxReal getAnalogAccel() const;
        PxReal getAnalogBrake() const;
        PxReal getAnalogHandbrake() const;
        PxReal getAnalogSteer() const;
        void setGearUp(const bool gearUpKeyPressed);
        void setGearDown(const bool gearDownKeyPressed);
        bool getGearUp() const;
        bool getGearDown() const;
};

void PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs
	(const PxVehicleKeySmoothingData& keySmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
	 const PxVehicleDrive4WRawInputData& rawInputData,
	 const PxReal timestep,
	 const bool isVehicleInAir,
	 PxVehicleDrive4W& focusVehicle);

void PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs
	(const PxVehiclePadSmoothingData& padSmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
	 const PxVehicleDrive4WRawInputData& rawInputData, 
	 const PxReal timestep,
	 const bool isVehicleInAir,
	 PxVehicleDrive4W& focusVehicle);

