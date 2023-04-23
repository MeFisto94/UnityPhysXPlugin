WRAPPER_CLASS(PxControllerDesc)
WRAPPER_CLASS(PxCapsuleControllerDesc)
class PxCapsuleControllerDesc : public PxControllerDesc
{
public:
	/**
	\brief constructor sets to default.
	*/
	PxCapsuleControllerDesc ();
	virtual ~PxCapsuleControllerDesc () {}

	/**
	\brief copy constructor.
	*/
	PxCapsuleControllerDesc(const PxCapsuleControllerDesc&);

	/**
	\brief assignment operator.
	*/
	PxCapsuleControllerDesc& operator=(const PxCapsuleControllerDesc&);

	/**
	\brief (re)sets the structure to the default.
	*/
	virtual void setToDefault();
	/**
	\brief returns true if the current settings are valid

	\return True if the descriptor is valid.
	*/
	virtual bool isValid() const;

	/**
	\brief The radius of the capsule

	<b>Default:</b> 0.0

	@see PxCapsuleController
	*/
	PxF32 radius;

	/**
	\brief The height of the controller

	<b>Default:</b> 0.0

	@see PxCapsuleController
	*/
	PxF32 height;

	/**
	\brief The climbing mode

	<b>Default:</b> PxCapsuleClimbingMode::eEASY

	@see PxCapsuleController
	*/
	//PxCapsuleClimbingMode::Enum		climbingMode;
};