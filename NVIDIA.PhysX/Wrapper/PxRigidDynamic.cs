//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------

namespace NVIDIA.PhysX {

public partial class PxRigidDynamic : PxRigidBody {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;

  internal PxRigidDynamic(global::System.IntPtr cPtr, bool cMemoryOwn) : base(NativePINVOKE.PxRigidDynamic_SWIGUpcast(cPtr), cMemoryOwn) {
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(PxRigidDynamic obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  internal static new PxRigidDynamic getWrapper(global::System.IntPtr cPtr, bool cMemoryOwn) {
      var wrapper = WrapperCache.find(cPtr);
      if (!(wrapper is PxRigidDynamic)) {
          wrapper = new PxRigidDynamic(cPtr, cMemoryOwn);
          WrapperCache.add(cPtr, wrapper);
      }
      return wrapper as PxRigidDynamic;
  }

  public void setKinematicTarget( PxTransform  destination) {
    NativePINVOKE.PxRigidDynamic_setKinematicTarget(swigCPtr,  destination.swigCPtr );
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public bool getKinematicTarget( ref PxTransform  target) {
    bool ret = NativePINVOKE.PxRigidDynamic_getKinematicTarget(swigCPtr,  target.swigCPtr );
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public bool isSleeping() {
    bool ret = NativePINVOKE.PxRigidDynamic_isSleeping(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setSleepThreshold(float threshold) {
    NativePINVOKE.PxRigidDynamic_setSleepThreshold(swigCPtr, threshold);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public float getSleepThreshold() {
    float ret = NativePINVOKE.PxRigidDynamic_getSleepThreshold(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setStabilizationThreshold(float threshold) {
    NativePINVOKE.PxRigidDynamic_setStabilizationThreshold(swigCPtr, threshold);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public float getStabilizationThreshold() {
    float ret = NativePINVOKE.PxRigidDynamic_getStabilizationThreshold(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setRigidDynamicLockFlags(PxRigidDynamicLockFlag flags) {
    NativePINVOKE.PxRigidDynamic_setRigidDynamicLockFlags(swigCPtr, (int)flags);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public PxRigidDynamicLockFlag getRigidDynamicLockFlags() {
    PxRigidDynamicLockFlag ret = (PxRigidDynamicLockFlag)NativePINVOKE.PxRigidDynamic_getRigidDynamicLockFlags(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setWakeCounter(float wakeCounterValue) {
    NativePINVOKE.PxRigidDynamic_setWakeCounter(swigCPtr, wakeCounterValue);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public float getWakeCounter() {
    float ret = NativePINVOKE.PxRigidDynamic_getWakeCounter(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void wakeUp() {
    NativePINVOKE.PxRigidDynamic_wakeUp(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public void putToSleep() {
    NativePINVOKE.PxRigidDynamic_putToSleep(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public void setSolverIterationCounts(uint minPositionIters, uint minVelocityIters) {
    NativePINVOKE.PxRigidDynamic_setSolverIterationCounts__SWIG_0(swigCPtr, minPositionIters, minVelocityIters);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public void setSolverIterationCounts(uint minPositionIters) {
    NativePINVOKE.PxRigidDynamic_setSolverIterationCounts__SWIG_1(swigCPtr, minPositionIters);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public void getSolverIterationCounts(out uint minPositionIters, out uint minVelocityIters) {
    NativePINVOKE.PxRigidDynamic_getSolverIterationCounts(swigCPtr, out minPositionIters, out minVelocityIters);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public float getContactReportThreshold() {
    float ret = NativePINVOKE.PxRigidDynamic_getContactReportThreshold(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setContactReportThreshold(float threshold) {
    NativePINVOKE.PxRigidDynamic_setContactReportThreshold(swigCPtr, threshold);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

}

}
