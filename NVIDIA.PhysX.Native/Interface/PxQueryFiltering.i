%include PxBatchQuery.i
%include Native.i

// struct PxQueryFlag
// {
// 	enum Enum
// 	{
// 		eSTATIC				= (1<<0),	//!< Traverse static shapes

// 		eDYNAMIC			= (1<<1),	//!< Traverse dynamic shapes

// 		ePREFILTER			= (1<<2),	//!< Run the pre-intersection-test filter (see #PxQueryFilterCallback::preFilter())

// 		ePOSTFILTER			= (1<<3),	//!< Run the post-intersection-test filter (see #PxQueryFilterCallback::postFilter())

// 		eANY_HIT			= (1<<4),	//!< Abort traversal as soon as any hit is found and return it via callback.block.
// 										//!< Helps query performance. Both eTOUCH and eBLOCK hitTypes are considered hits with this flag.

// 		eNO_BLOCK			= (1<<5),	//!< All hits are reported as touching. Overrides eBLOCK returned from user filters with eTOUCH.
// 										//!< This is also an optimization hint that may improve query performance.

// 		eRESERVED			= (1<<15)	//!< Reserved for internal use
// 	};
// };

typedef PxFlags<PxQueryFlag::Enum,PxU16> PxQueryFlags;
PX_FLAGS_OPERATORS(PxQueryFlag::Enum,PxU16);
typedef PxFlags<PxHitFlag,PxU16> PxHitFlags;
PX_FLAGS_OPERATORS(PxHitFlag,PxU16);

SIMPLIFY_ENUM(PxQueryHitType,
        eNONE	= 0,	//!< the query should ignore this shape
		eTOUCH	= 1,	//!< a hit on the shape touches the intersection geometry of the query but does not block it
		eBLOCK	= 2		//!< a hit on the shape blocks the query (does not block overlap queries)
);

// struct PxQueryFilterData
// {
// 	/** \brief default constructor */
// 	explicit PxQueryFilterData() : flags(PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC)		{}

// 	/** \brief constructor to set both filter data and filter flags */
// 	explicit PxQueryFilterData(const PxFilterData& fd, PxQueryFlags f) : data(fd), flags(f)	{}

// 	/** \brief constructor to set filter flags only */
// 	explicit PxQueryFilterData(PxQueryFlags f) : flags(f)										{}

// 	PxFilterData	data;		//!< Filter data associated with the scene query
// 	PxQueryFlags	flags;		//!< Filter flags (see #PxQueryFlags)
// };

%feature("csdirectordelegatemodifiers") "public"
%feature("director") PxQueryFilterCallback;
class PxQueryFilterCallback
{
public:

	%typemap (cstype) PxHitFlags "PxHitFlag"
	%typemap (imtype) PxHitFlags "PxHitFlag"
	// TODO: Conversion proxy -> wrapper
	%typemap (csout) PxHitFlags "test $csinput"
	/**
	\brief This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.

	\param[in] filterData custom filter data specified as the query's filterData.data parameter.
	\param[in] shape A shape that has not yet passed the exact intersection test.
	\param[in] actor The shape's actor.
	\param[in,out] queryFlags scene query flags from the query's function call (only flags from PxHitFlag::eMODIFIABLE_FLAGS bitmask can be modified)
	\return the updated type for this hit  (see #PxQueryHitType)
	*/
	virtual PxQueryHitType::Enum preFilter(const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags/*PxHitFlag::Enum*/ queryFlags) = 0;
    /*%extend { virtual PxQueryHitType::Enum preFilter(const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlag::Enum queryFlags) = 0; %{
        physx::PxQueryHitType::Enum physx_PxQueryFilterCallback_preFilter(const physx::PxQueryFilterCallback thizz, const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlag::Enum queryFlags) {
            thizz->preFilter(filterData, shape, actor, (physx::PxHitFlags)(uint32_t)queryFlags);
        }
    %}}*/

	/**
	\brief This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.

	\param[in] filterData custom filter data of the query
	\param[in] hit Scene query hit information. faceIndex member is not valid for overlap queries. For sweep and raycast queries the hit information can be cast to #PxSweepHit and #PxRaycastHit respectively.
	\return the updated hit type for this hit  (see #PxQueryHitType)
	*/
	virtual PxQueryHitType::Enum postFilter(const PxFilterData& filterData, const PxQueryHit& hit) = 0;

	/**
	\brief virtual destructor
	*/
	virtual ~PxQueryFilterCallback() {}
};

// Required, otherwise the delegate inside is less accessible than the other PhysX classes
// TODO: Maybe someone knows I can emit the imclasscode outside of the NativePINVOKE class, but within reach of it?
// I'd probably work by defining a class %inline, but that's not clean IMO
%pragma(csharp) imclassclassmodifiers="public class"

%pragma(csharp) imclasscode=%{
  public delegate PxQueryHitType PxBatchQueryPreFilterShaderDelegate(PxFilterData queryFilterData, PxFilterData objectFilterData,
	global::System.IntPtr constantBlock, uint constantBlockSize,
	PxHitFlag hitFlags);
  public delegate PxQueryHitType PxBatchQueryPostFilterShaderDelegate(PxFilterData queryFilterData, PxFilterData objectFilterData,
	global::System.IntPtr constantBlock, uint constantBlockSize,
	PxQueryHit hit);
%}

typedef PxQueryHitType::Enum (*PxBatchQueryPreFilterShader)(
	PxFilterData queryFilterData, PxFilterData objectFilterData,
	const void* constantBlock, PxU32 constantBlockSize,
	PxHitFlags& hitFlags);

typedef PxQueryHitType::Enum (*PxBatchQueryPostFilterShader)(
	PxFilterData queryFilterData, PxFilterData objectFilterData,
	const void* constantBlock, PxU32 constantBlockSize,
	const PxQueryHit& hit);

%typemap (cstype) physx::PxBatchQueryPreFilterShader "NativePINVOKE.PxBatchQueryPreFilterShaderDelegate"
%typemap (imtype) physx::PxBatchQueryPreFilterShader "PxBatchQueryPreFilterShaderDelegate"
%typemap (csin) physx::PxBatchQueryPreFilterShader "$csinput"
%typemap (csvarout, excode=SWIGEXCODE) physx::PxBatchQueryPreFilterShader %{
  get { $excode;return $imcall; }
%}

%typemap (cstype) physx::PxBatchQueryPostFilterShader "NativePINVOKE.PxBatchQueryPostFilterShaderDelegate"
%typemap (imtype) physx::PxBatchQueryPostFilterShader "PxBatchQueryPostFilterShaderDelegate"
%typemap (csin) physx::PxBatchQueryPostFilterShader "$csinput"
%typemap (csvarout, excode=SWIGEXCODE) physx::PxBatchQueryPostFilterShader %{
  get { $excode;return $imcall; }
%}
