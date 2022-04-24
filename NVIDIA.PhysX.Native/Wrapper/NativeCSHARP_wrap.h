/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * This file is not intended to be easily readable and contains a number of
 * coding conventions designed to improve portability and efficiency. Do not make
 * changes to this file unless you know what you are doing--modify the SWIG
 * interface file instead.
 * ----------------------------------------------------------------------------- */

#ifndef SWIG_Native_WRAP_H_
#define SWIG_Native_WRAP_H_

class SwigDirector_PxSimulationEventCallback : public physx::wrap::PxSimulationEventCallback, public Swig::Director {

public:
    SwigDirector_PxSimulationEventCallback();
    virtual void onConstraintBreak(physx::PxConstraintInfoList const &constraints);
    virtual void onWake(physx::PxActorList const &actors);
    virtual void onSleep(physx::PxActorList const &actors);
    virtual void onContact(physx::PxContactPairHeader const &pairHeader, physx::PxContactPairList const &pairs);
    virtual void onTrigger(physx::PxTriggerPairList const &pairs);
    virtual void onAdvance(physx::PxRigidBodyList const &bodyBuffer, physx::PxTransformList const &poseBuffer);
    virtual ~SwigDirector_PxSimulationEventCallback();

    typedef void (SWIGSTDCALL* SWIG_Callback0_t)(void *);
    typedef void (SWIGSTDCALL* SWIG_Callback1_t)(void *);
    typedef void (SWIGSTDCALL* SWIG_Callback2_t)(void *);
    typedef void (SWIGSTDCALL* SWIG_Callback3_t)(void *, void *);
    typedef void (SWIGSTDCALL* SWIG_Callback4_t)(void *);
    typedef void (SWIGSTDCALL* SWIG_Callback5_t)(void *, void *);
    void swig_connect_director(SWIG_Callback0_t callbackonConstraintBreak, SWIG_Callback1_t callbackonWake, SWIG_Callback2_t callbackonSleep, SWIG_Callback3_t callbackonContact, SWIG_Callback4_t callbackonTrigger, SWIG_Callback5_t callbackonAdvance);

private:
    SWIG_Callback0_t swig_callbackonConstraintBreak;
    SWIG_Callback1_t swig_callbackonWake;
    SWIG_Callback2_t swig_callbackonSleep;
    SWIG_Callback3_t swig_callbackonContact;
    SWIG_Callback4_t swig_callbackonTrigger;
    SWIG_Callback5_t swig_callbackonAdvance;
    void swig_init_callbacks();
};

class SwigDirector_PxSimulationFilterCallback : public physx::wrap::PxSimulationFilterCallback, public Swig::Director {

public:
    SwigDirector_PxSimulationFilterCallback();
    virtual void pairLost(physx::PxU32 pairID, bool objectRemoved, physx::PxFilterObjectAttributes attributes0, physx::PxFilterData const &filterData0, physx::PxFilterObjectAttributes attributes1, physx::PxFilterData const &filterData1);
    virtual ~SwigDirector_PxSimulationFilterCallback();

    typedef void (SWIGSTDCALL* SWIG_Callback0_t)(unsigned int, unsigned int, unsigned int,  physx::PxFilterData* , unsigned int,  physx::PxFilterData* );
    void swig_connect_director(SWIG_Callback0_t callbackpairLost);

private:
    SWIG_Callback0_t swig_callbackpairLost;
    void swig_init_callbacks();
};

class SwigDirector_PxBroadPhaseCallback : public physx::PxBroadPhaseCallback, public Swig::Director {

public:
    SwigDirector_PxBroadPhaseCallback();
    virtual ~SwigDirector_PxBroadPhaseCallback();
    virtual void onObjectOutOfBounds(physx::PxShape &shape, physx::PxActor &actor);
    virtual void onObjectOutOfBounds(physx::PxAggregate &aggregate);

    typedef void (SWIGSTDCALL* SWIG_Callback0_t)(void *, void *);
    typedef void (SWIGSTDCALL* SWIG_Callback1_t)(void *);
    void swig_connect_director(SWIG_Callback0_t callbackonObjectOutOfBounds__SWIG_0, SWIG_Callback1_t callbackonObjectOutOfBounds__SWIG_1);

private:
    SWIG_Callback0_t swig_callbackonObjectOutOfBounds__SWIG_0;
    SWIG_Callback1_t swig_callbackonObjectOutOfBounds__SWIG_1;
    void swig_init_callbacks();
};

class SwigDirector_PxErrorCallback : public physx::PxErrorCallback, public Swig::Director {

public:
    SwigDirector_PxErrorCallback();
    virtual ~SwigDirector_PxErrorCallback();
    virtual void reportError(physx::PxErrorCode::Enum code, char const *message, char const *file, int line);

    typedef void (SWIGSTDCALL* SWIG_Callback0_t)(int, char *, char *, int);
    void swig_connect_director(SWIG_Callback0_t callbackreportError);

private:
    SWIG_Callback0_t swig_callbackreportError;
    void swig_init_callbacks();
};

class SwigDirector_PxQueryFilterCallback : public physx::PxQueryFilterCallback, public Swig::Director {

public:
    SwigDirector_PxQueryFilterCallback();
    virtual physx::PxQueryHitType::Enum preFilter(physx::PxFilterData const &filterData, physx::PxShape const *shape, physx::PxRigidActor const *actor, physx::PxHitFlags queryFlags);
    virtual physx::PxQueryHitType::Enum postFilter(physx::PxFilterData const &filterData, physx::PxQueryHit const &hit);
    virtual ~SwigDirector_PxQueryFilterCallback();

    typedef int (SWIGSTDCALL* SWIG_Callback0_t)( physx::PxFilterData* , void *, void *, void *);
    typedef int (SWIGSTDCALL* SWIG_Callback1_t)( physx::PxFilterData* , void *);
    void swig_connect_director(SWIG_Callback0_t callbackpreFilter, SWIG_Callback1_t callbackpostFilter);

private:
    SWIG_Callback0_t swig_callbackpreFilter;
    SWIG_Callback1_t swig_callbackpostFilter;
    void swig_init_callbacks();
};


#endif
