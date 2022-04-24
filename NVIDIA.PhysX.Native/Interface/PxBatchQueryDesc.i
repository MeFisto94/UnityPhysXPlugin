%include PxQueryFiltering.i

/**
\brief Batched query status.

\deprecated The batched query feature has been deprecated in PhysX version 3.4
*/
struct PxBatchQueryStatus
{
	enum Enum
	{
		/**
		\brief This is the initial state before a query starts.
		*/
		ePENDING = 0,

		/**
		\brief The query is finished; results have been written into the result and hit buffers.
		*/
		eSUCCESS,

		/**
		\brief The query results were incomplete due to touch hit buffer overflow. Blocking hit is still correct.
		*/
		eOVERFLOW
	};
};

/**
\brief Generic struct for receiving results of single query in a batch. Gets templated on hit type PxRaycastHit, PxSweepHit or PxOverlapHit.

\deprecated The batched query feature has been deprecated in PhysX version 3.4
*/
template<typename HitType>
struct PxBatchQueryResult
{
	HitType			block;			//!< Holds the closest blocking hit for a single query in a batch. Only valid if hasBlock is true.
	HitType*		touches;		//!< This pointer will either be set to NULL for 0 nbTouches or will point
									//!< into the user provided batch query results buffer specified in PxBatchQueryDesc.
	PxU32			nbTouches;		//!< Number of touching hits returned by this query, works in tandem with touches pointer.
	void*			userData;		//!< Copy of the userData pointer specified in the corresponding query.
	PxU8			queryStatus;	//!< Takes on values from PxBatchQueryStatus::Enum.
	bool			hasBlock;		//!< True if there was a blocking hit.
	PxU16			pad;			//!< pads the struct to 16 bytes.

	/** \brief Computes the number of any hits in this result, blocking or touching. */
	PxU32				getNbAnyHits() const				{ return nbTouches + (hasBlock ? 1 : 0); }

	/** \brief Convenience iterator used to access any hits in this result, blocking or touching. */
	const HitType&	getAnyHit(const PxU32 index) const	{ PX_ASSERT(index < nbTouches + (hasBlock ? 1 : 0));
																		return index < nbTouches ? touches[index] : block; }
};

//FLAT_STRUCT(physx::PxRaycastQueryResult, PxRaycastQueryResult, public PxRaycastHit block; public System.IntPtr touches; public uint nbTouches; System.IntPtr userData; byte queryStatus; bool hasBlock; ushort pad;)
//
// Here, we ignore SWIG magic and manually define the struct on the C# side.
// Unfortunately, every component needs to be a struct then though, so in this case it's transitive on PxRaycastHit etc.
//struct PxRaycastQueryResult {};


/** \brief Convenience typedef for the result of a batched raycast query. */
typedef PxBatchQueryResult<PxRaycastHit>	PxRaycastQueryResult;

/** \brief Convenience typedef for the result of a batched sweep query. */
typedef PxBatchQueryResult<PxSweepHit>		PxSweepQueryResult;

/** \brief Convenience typedef for the result of a batched overlap query. */
typedef PxBatchQueryResult<PxOverlapHit>	PxOverlapQueryResult;

%template(PxSweepQueryResult) PxBatchQueryResult<PxSweepHit>;
%template(PxOverlapQueryResult) PxBatchQueryResult<PxOverlapHit>;
%template(PxRaycastQueryResult) PxBatchQueryResult<PxRaycastHit>;


/**
\brief Struct for #PxBatchQuery memory pointers.

\deprecated The batched query feature has been deprecated in PhysX version 3.4
 
@see PxBatchQuery PxBatchQueryDesc
*/
struct PxBatchQueryMemory
 {
 	/**
	\brief The pointer to the user-allocated buffer for results of raycast queries in corresponding order of issue
 
 	\note The size should be large enough to fit the number of expected raycast queries.

 	@see PxRaycastQueryResult 
 	*/
 	PxRaycastQueryResult*			userRaycastResultBuffer;
 
 	/**
 	\brief The pointer to the user-allocated buffer for raycast touch hits.
 	\note The size of this buffer should be large enough to store PxRaycastHit.
 	If the buffer is too small to store hits, the related PxRaycastQueryResult.queryStatus will be set to eOVERFLOW
 
 	*/
 	PxRaycastHit*					userRaycastTouchBuffer;
 
 	/**
 	\brief The pointer to the user-allocated buffer for results of sweep queries in corresponding order of issue
 
 	\note The size should be large enough to fit the number of expected sweep queries.
 
 	@see PxRaycastQueryResult 
 	*/
 	PxSweepQueryResult*				userSweepResultBuffer;

 	/**
 	\brief The pointer to the user-allocated buffer for sweep hits.
 	\note The size of this buffer should be large enough to store PxSweepHit. 
 	If the buffer is too small to store hits, the related PxSweepQueryResult.queryStatus will be set to eOVERFLOW
 
 	*/
 	PxSweepHit*						userSweepTouchBuffer;
 
 	/**
 	\brief The pointer to the user-allocated buffer for results of overlap queries in corresponding order of issue
 
 	\note The size should be large enough to fit the number of expected overlap queries.
 
 	@see PxRaycastQueryResult 
 	*/
 	PxOverlapQueryResult*			userOverlapResultBuffer;
 
 	/**
 	\brief The pointer to the user-allocated buffer for overlap hits.
 	\note The size of this buffer should be large enough to store the hits returned. 
 	If the buffer is too small to store hits, the related PxOverlapQueryResult.queryStatus will be set to eABORTED

 	*/
 	PxOverlapHit*					userOverlapTouchBuffer;
 
 	/** \brief Capacity of the user-allocated userRaycastTouchBuffer in elements */
 	PxU32							raycastTouchBufferSize;
 
 	/** \brief Capacity of the user-allocated userSweepTouchBuffer in elements */
 	PxU32							sweepTouchBufferSize;
 
 	/** \brief Capacity of the user-allocated userOverlapTouchBuffer in elements */
 	PxU32							overlapTouchBufferSize;

 	/** \return Capacity of the user-allocated userRaycastResultBuffer in elements (max number of raycast() calls before execute() call) */
	PxU32			getMaxRaycastsPerExecute() const	{ return raycastResultBufferSize; }

 	/** \return Capacity of the user-allocated userSweepResultBuffer in elements (max number of sweep() calls before execute() call) */
	PxU32			getMaxSweepsPerExecute() const		{ return sweepResultBufferSize; }

 	/** \return Capacity of the user-allocated userOverlapResultBuffer in elements (max number of overlap() calls before execute() call) */
	PxU32			getMaxOverlapsPerExecute() const	{ return overlapResultBufferSize; }

	PxBatchQueryMemory(PxU32 raycastResultBufferSize_, PxU32 sweepResultBufferSize_, PxU32 overlapResultBufferSize_) :
		userRaycastResultBuffer	(NULL),
		userRaycastTouchBuffer	(NULL),
		userSweepResultBuffer	(NULL),
		userSweepTouchBuffer	(NULL),
		userOverlapResultBuffer	(NULL),
		userOverlapTouchBuffer	(NULL),
		raycastTouchBufferSize	(0),
		sweepTouchBufferSize	(0),
		overlapTouchBufferSize	(0),
		raycastResultBufferSize	(raycastResultBufferSize_),
		sweepResultBufferSize	(sweepResultBufferSize_),
		overlapResultBufferSize	(overlapResultBufferSize_)
	{
	}

protected:
 	PxU32							raycastResultBufferSize;
 	PxU32							sweepResultBufferSize;
 	PxU32							overlapResultBufferSize;
};

/**
\brief Descriptor class for #PxBatchQuery.

\deprecated The batched query feature has been deprecated in PhysX version 3.4

@see PxBatchQuery PxSceneQueryExecuteMode
*/

class PxBatchQueryDesc
{
public:

	/**
	\brief Shared global filter data which will get passed into the filter shader.

	\note The provided data will get copied to internal buffers and this copy will be used for filtering calls.

	<b>Default:</b> NULL

	@see PxSimulationFilterShader
	*/
	void*							filterShaderData;

	/**
	\brief Size (in bytes) of the shared global filter data #filterShaderData.

	<b>Default:</b> 0

	@see PxSimulationFilterShader filterShaderData
	*/
	PxU32							filterShaderDataSize;

	/**
	\brief The custom preFilter shader to use for filtering.

	@see PxBatchQueryPreFilterShader PxDefaultPreFilterShader
	*/
	PxBatchQueryPreFilterShader		preFilterShader;

		/**
	\brief The custom postFilter shader to use for filtering.

	@see PxBatchQueryPostFilterShader PxDefaultPostFilterShader
	*/
	PxBatchQueryPostFilterShader	postFilterShader;	

	/**
	\brief User memory buffers for the query.

	@see PxBatchQueryMemory
	*/
	PxBatchQueryMemory				queryMemory;

	/**
	\brief Construct a batch query with specified maximum number of queries per batch.

	If the number of raycasts/sweeps/overlaps per execute exceeds the limit, the query will be discarded with a warning.

	\param maxRaycastsPerExecute	Maximum number of raycast() calls allowed before execute() call.
									This has to match the amount of memory allocated for PxBatchQueryMemory::userRaycastResultBuffer.
	\param maxSweepsPerExecute	Maximum number of sweep() calls allowed before execute() call.
									This has to match the amount of memory allocated for PxBatchQueryMemory::userSweepResultBuffer.
	\param maxOverlapsPerExecute	Maximum number of overlap() calls allowed before execute() call.
									This has to match the amount of memory allocated for PxBatchQueryMemory::userOverlapResultBuffer.
	*/
	PxBatchQueryDesc(PxU32 maxRaycastsPerExecute, PxU32 maxSweepsPerExecute, PxU32 maxOverlapsPerExecute);
	bool					isValid() const;
};
