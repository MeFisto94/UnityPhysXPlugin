%include PxQueryFiltering.i

WRAPPER_CLASS(PxController)
WRAPPER_CLASS(PxControllerDesc)

class PxControllerDesc {
	public:
	/**
	\brief returns true if the current settings are valid

	\return True if the descriptor is valid.
	*/
	//virtual	bool isValid() const;

	/**
	\brief Returns the character controller type

	\return The controllers type.

	@see PxControllerType PxCapsuleControllerDesc PxBoxControllerDesc
	*/
	//PxControllerShapeType::Enum		getType()		const	{ return mType;		}

	/**
	\brief The position of the character

	\note The character's initial position must be such that it does not overlap the static geometry.

	<b>Default:</b> Zero
	*/
	PxExtendedVec3						position;

	/**
	\brief Specifies the 'up' direction

	In order to provide stepping functionality the SDK must be informed about the up direction.

	<b>Default:</b> (0, 1, 0)

	*/
	PxVec3								upDirection;

	/**
	\brief The maximum slope which the character can walk up.

	In general it is desirable to limit where the character can walk, in particular it is unrealistic
	for the character to be able to climb arbitary slopes.

	The limit is expressed as the cosine of desired limit angle. A value of 0 disables this feature.

	\warning It is currently enabled for static actors only (not for dynamic/kinematic actors), and not supported for spheres or capsules.

	<b>Default:</b> 0.707

	@see upDirection invisibleWallHeight maxJumpHeight
	*/
	PxF32								slopeLimit;

	/**
	\brief Height of invisible walls created around non-walkable triangles

	The library can automatically create invisible walls around non-walkable triangles defined
	by the 'slopeLimit' parameter. This defines the height of those walls. If it is 0.0, then
	no extra triangles are created.

	<b>Default:</b> 0.0

	@see upDirection slopeLimit maxJumpHeight
	*/
	PxF32								invisibleWallHeight;

	/**
	\brief Maximum height a jumping character can reach

	This is only used if invisible walls are created ('invisibleWallHeight' is non zero).

	When a character jumps, the non-walkable triangles he might fly over are not found
	by the collision queries (since the character's bounding volume does not touch them).
	Thus those non-walkable triangles do not create invisible walls, and it is possible
	for a jumping character to land on a non-walkable triangle, while he wouldn't have
	reached that place by just walking.

	The 'maxJumpHeight' variable is used to extend the size of the collision volume
	downward. This way, all the non-walkable triangles are properly found by the collision
	queries and it becomes impossible to 'jump over' invisible walls.

	If the character in your game can not jump, it is safe to use 0.0 here. Otherwise it
	is best to keep this value as small as possible, since a larger collision volume
	means more triangles to process.

	<b>Default:</b> 0.0

	@see upDirection slopeLimit invisibleWallHeight
	*/
	PxF32								maxJumpHeight;

	/**
	\brief The contact offset used by the controller.

	Specifies a skin around the object within which contacts will be generated.
	Use it to avoid numerical precision issues.

	This is dependant on the scale of the users world, but should be a small, positive 
	non zero value.

	<b>Default:</b> 0.1
	*/
	PxF32								contactOffset;

	/**
	\brief Defines the maximum height of an obstacle which the character can climb.

	A small value will mean that the character gets stuck and cannot walk up stairs etc, 
	a value which is too large will mean that the character can climb over unrealistically 
	high obstacles.

	<b>Default:</b> 0.5

	@see upDirection 
	*/
	PxF32								stepOffset;

	/**
	\brief Density of underlying kinematic actor

	The CCT creates a PhysX's kinematic actor under the hood. This controls its density.

	<b>Default:</b> 10.0
	*/
	PxF32								density;

	/**
	\brief Scale coefficient for underlying kinematic actor

	The CCT creates a PhysX's kinematic actor under the hood. This controls its scale factor.
	This should be a number a bit smaller than 1.0.

	<b>Default:</b> 0.8
	*/
	PxF32								scaleCoeff;

	/**
	\brief Cached volume growth

	Amount of space around the controller we cache to improve performance. This is a scale factor
	that should be higher than 1.0f but not too big, ideally lower than 2.0f.

	<b>Default:</b> 1.5
	*/
	PxF32								volumeGrowth;

	/**
	\brief Specifies a user report callback.

	This report callback is called when the character collides with shapes and other characters.

	Setting this to NULL disables the callback.

	<b>Default:</b> NULL

	@see PxUserControllerHitReport
	*/
	//PxUserControllerHitReport*			reportCallback;

	/**
	\brief Specifies a user behavior callback.

	This behavior callback is called to customize the controller's behavior w.r.t. touched shapes.

	Setting this to NULL disables the callback.

	<b>Default:</b> NULL

	@see PxControllerBehaviorCallback
	*/
	//PxControllerBehaviorCallback*		behaviorCallback;

	/**
	\brief The non-walkable mode controls if a character controller slides or not on a non-walkable part.

	This is only used when slopeLimit is non zero.

	<b>Default:</b> PxControllerNonWalkableMode::ePREVENT_CLIMBING

	@see PxControllerNonWalkableMode
	*/
	//PxControllerNonWalkableMode::Enum	nonWalkableMode;

	/**
	\brief The material for the actor associated with the controller.
	
	The controller internally creates a rigid body actor. This parameter specifies the material of the actor.

	<b>Default:</b> NULL

	@see PxMaterial
	*/
	PxMaterial*							material;

	/**
	\brief Use a deletion listener to get informed about released objects and clear internal caches if needed.

	If a character controller registers a deletion listener, it will get informed about released objects. That allows the
	controller to invalidate cached data that connects to a released object. If a deletion listener is not
	registered, PxController::invalidateCache has to be called manually after objects have been released.

	@see PxController::invalidateCache

	<b>Default:</b> true
	*/
	bool								registerDeletionListener;

	/**
	\brief User specified data associated with the controller.

	<b>Default:</b> NULL
	*/
	//void*								userData;
	// seems to be handled by WRAPPER_CLASS
    protected:
	    PxControllerDesc() {}
	    virtual ~PxControllerDesc() {}
};

class PxController;

class PxControllerFilterCallback
{
public:
	virtual ~PxControllerFilterCallback(){}

	/**
	\brief Filtering method for CCT-vs-CCT.

	\param[in] a	First CCT
	\param[in] b	Second CCT
	\return true to keep the pair, false to filter it out
	*/
	virtual bool filter(const PxController& a, const PxController& b) = 0;
};

/**
\brief specifies which sides a character is colliding with.
*/
SIMPLIFY_ENUM(PxControllerCollisionFlag, eCOLLISION_SIDES	= (1<<0),	//!< Character is colliding to the sides.
		eCOLLISION_UP		= (1<<1),	//!< Character has collision above.
		eCOLLISION_DOWN		= (1<<2)	//!< Character has collision below.
)

/**
\brief Filtering data for "move" call.

This class contains all filtering-related parameters for the PxController::move() call.

Collisions between a CCT and the world are filtered using the mFilterData, mFilterCallback and mFilterFlags
members. These parameters are internally passed to PxScene::overlap() to find objects touched by the CCT.
Please refer to the PxScene::overlap() documentation for details.

Collisions between a CCT and another CCT are filtered using the mCCTFilterCallback member. If this filter
callback is not defined, none of the CCT-vs-CCT collisions are filtered, and each CCT will collide against
all other CCTs.

\note PxQueryFlag::eANY_HIT and PxQueryFlag::eNO_BLOCK are ignored in mFilterFlags.

@see PxController.move() PxControllerFilterCallback
*/
class PxControllerFilters
{
	public:
        PxControllerFilters(const PxFilterData* filterData=NULL, PxQueryFilterCallback* cb=NULL, PxControllerFilterCallback* cctFilterCb=NULL)/* :
									mFilterData			(filterData),
									mFilterCallback		(cb),
									mFilterFlags		(PxQueryFlag::eSTATIC|PxQueryFlag::eDYNAMIC|PxQueryFlag::ePREFILTER),
									mCCTFilterCallback	(cctFilterCb)*/
								{}

	// CCT-vs-shapes:
	const PxFilterData*			mFilterData;			//!< Data for internal PxQueryFilterData structure. Passed to PxScene::overlap() call.
														//!< This can be NULL, in which case a default PxFilterData is used.
	PxQueryFilterCallback*		mFilterCallback;		//!< Custom filter logic (can be NULL). Passed to PxScene::overlap() call.
	//PxQueryFlags				mFilterFlags;			//!< Flags for internal PxQueryFilterData structure. Passed to PxScene::overlap() call.
	// CCT-vs-CCT:
	PxControllerFilterCallback*	mCCTFilterCallback;		//!< CCT-vs-CCT filter callback. If NULL, all CCT-vs-CCT collisions are kept.
};

class PxController {
    protected:
	    PxController() {}
	    virtual ~PxController() {}
    public:
        //PxShape* createShape(const PxGeometry& geometry, const PxMaterial& material, bool isExclusive = false, PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE);
        //%extend { PxShape* createShape(const PxGeometry& geometry, const PxMaterial& material, bool isExclusive = false, PxShapeFlag::Enum shapeFlags = (physx::PxShapeFlag::Enum)(uint32_t)(physx::PxShapeFlag::eVISUALIZATION|physx::PxShapeFlag::eSCENE_QUERY_SHAPE|physx::PxShapeFlag::eSIMULATION_SHAPE)) { return self->createShape(geometry, material, isExclusive, shapeFlags); }}
        //virtual		PxControllerCollisionFlags	move(const PxVec3& disp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles=NULL) = 0;
        %extend { /*physx::*/PxControllerCollisionFlag::Enum move(const PxVec3& disp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles=NULL) { return (physx::PxControllerCollisionFlag::Enum)(uint32_t)self->move(disp, minDist, elapsedTime, filters, obstacles); }}

	/**
	\brief Return the type of controller

	@see PxControllerType
	*/
	// virtual		PxControllerShapeType::Enum	getType()		const			= 0;

	/**
	\brief Releases the controller.
	*/
	virtual		void					release() = 0;

	/**
	\brief Sets controller's position.

	The position controlled by this function is the center of the collision shape.

	\warning This is a 'teleport' function, it doesn't check for collisions.
	\warning The character's position must be such that it does not overlap the static geometry.

	To move the character under normal conditions use the #move() function.

	\param[in] position The new (center) positon for the controller.
	\return Currently always returns true.

	@see PxControllerDesc.position getPosition() getFootPosition() setFootPosition() move()
	*/
	virtual		bool					setPosition(const PxExtendedVec3& position) = 0;

	/**
	\brief Retrieve the raw position of the controller.

	The position retrieved by this function is the center of the collision shape. To retrieve the bottom position of the shape,
	a.k.a. the foot position, use the getFootPosition() function.

	The position is updated by calls to move(). Calling this method without calling
	move() will return the last position or the initial position of the controller.

	\return The controller's center position

	@see PxControllerDesc.position setPosition() getFootPosition() setFootPosition() move()
	*/
	virtual		const PxExtendedVec3&	getPosition()			const	= 0;

	/**
	\brief Set controller's foot position.

	The position controlled by this function is the bottom of the collision shape, a.k.a. the foot position.

	\note The foot position takes the contact offset into account

	\warning This is a 'teleport' function, it doesn't check for collisions.

	To move the character under normal conditions use the #move() function.

	\param[in] position The new (bottom) positon for the controller.
	\return Currently always returns true.

	@see PxControllerDesc.position setPosition() getPosition() getFootPosition() move()
	*/
	virtual		bool					setFootPosition(const PxExtendedVec3& position) = 0;

	/**
	\brief Retrieve the "foot" position of the controller, i.e. the position of the bottom of the CCT's shape.

	\note The foot position takes the contact offset into account

	\return The controller's foot position

	@see PxControllerDesc.position setPosition() getPosition() setFootPosition() move()
	*/
	virtual		PxExtendedVec3			getFootPosition()		const	= 0;

	/**
	\brief Get the rigid body actor associated with this controller (see PhysX documentation). 
	The behavior upon manually altering this actor is undefined, you should primarily 
	use it for reading const properties.

	\return the actor associated with the controller.
	*/
	virtual		PxRigidDynamic*			getActor()				const	= 0;

	/**
	\brief The step height.

	\param[in] offset The new step offset for the controller.

	@see PxControllerDesc.stepOffset
	*/
	virtual	    void					setStepOffset(const PxF32 offset) =0;

	/**
	\brief Retrieve the step height.

	\return The step offset for the controller.

	@see setStepOffset()
	*/
	virtual	    PxF32					getStepOffset()						const		=0;

	/**
	\brief Sets the non-walkable mode for the CCT.

	\param[in] flag The new value of the non-walkable mode.

	\see PxControllerNonWalkableMode
	*/
	// virtual		void						setNonWalkableMode(PxControllerNonWalkableMode::Enum flag)	= 0;

	/**
	\brief Retrieves the non-walkable mode for the CCT.

	\return The current non-walkable mode.

	\see PxControllerNonWalkableMode
	*/
	//virtual		PxControllerNonWalkableMode::Enum	getNonWalkableMode()				const		= 0;

	/**
	\brief Retrieve the contact offset.

	\return The contact offset for the controller.

	@see PxControllerDesc.contactOffset
	*/
	virtual	    PxF32					getContactOffset()					const		=0;

	/**
	\brief Sets the contact offset.

	\param[in] offset	The contact offset for the controller.

	@see PxControllerDesc.contactOffset
	*/
	virtual	    void					setContactOffset(PxF32 offset)					=0;

	/**
	\brief Retrieve the 'up' direction.

	\return The up direction for the controller.

	@see PxControllerDesc.upDirection
	*/
	virtual		PxVec3					getUpDirection()					const		=0;

	/**
	\brief Sets the 'up' direction.

	\param[in] up The up direction for the controller.

	@see PxControllerDesc.upDirection
	*/
	virtual		void					setUpDirection(const PxVec3& up)				=0;

	/**
	\brief Retrieve the slope limit.

	\return The slope limit for the controller.

	@see PxControllerDesc.slopeLimit
	*/
	virtual	    PxF32					getSlopeLimit()						const		=0;

	/**
	\brief Sets the slope limit.

	\note	This feature can not be enabled at runtime, i.e. if the slope limit is zero when creating the CCT
	(which disables the feature) then changing the slope limit at runtime will not have any effect, and the call
	will be ignored.

	\param[in]	slopeLimit	The slope limit for the controller.

	@see PxControllerDesc.slopeLimit
	*/
	virtual	    void					setSlopeLimit(PxF32 slopeLimit)					=0;

	/**
	\brief Flushes internal geometry cache.
	
	The character controller uses caching in order to speed up collision testing. The cache is
	automatically flushed when a change to static objects is detected in the scene. For example when a
	static shape is added, updated, or removed from the scene, the cache is automatically invalidated.
	
	However there may be situations that cannot be automatically detected, and those require manual
	invalidation of the cache. Currently the user must call this when the filtering behavior changes (the
	PxControllerFilters parameter of the PxController::move call).  While the controller in principle
	could detect a change in these parameters, it cannot detect a change in the behavior of the filtering 
	function.

	@see PxController.move
	*/
	virtual		void					invalidateCache()			= 0;

	/**
	\brief Retrieve the scene associated with the controller.

	\return The physics scene
	*/
	virtual		PxScene*				getScene()						= 0;

	/**
	\brief Returns the user data associated with this controller.

	\return The user pointer associated with the controller.

	@see PxControllerDesc.userData
	*/
	virtual		void*					getUserData()		const		= 0;

	/**
	\brief Sets the user data associated with this controller.

	\param[in] userData The user pointer associated with the controller.

	@see PxControllerDesc.userData
	*/
	virtual		void					setUserData(void* userData)		= 0;

	/**
	\brief Returns information about the controller's internal state.

	\param[out] state The controller's internal state

	@see PxControllerState
	*/
	// virtual		void					getState(PxControllerState& state)	const		= 0;

	/**
	\brief Returns the controller's internal statistics.

	\param[out] stats The controller's internal statistics

	@see PxControllerStats
	*/
	// virtual		void					getStats(PxControllerStats& stats)	const		= 0;

	/**
	\brief Resizes the controller.

	This function attempts to resize the controller to a given size, while making sure the bottom
	position of the controller remains constant. In other words the function modifies both the
	height and the (center) position of the controller. This is a helper function that can be used
	to implement a 'crouch' functionality for example.

	\param[in] height Desired controller's height
	*/
	virtual		void					resize(PxReal height)	= 0;
};

