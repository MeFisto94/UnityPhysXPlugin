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

public partial class PxVehicleTireData {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal PxVehicleTireData(global::System.IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(PxVehicleTireData obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  ~PxVehicleTireData() {
    destroy();
  }

  public virtual void destroy() {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          NativePINVOKE.delete_PxVehicleTireData(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      global::System.GC.SuppressFinalize(this);
    }
  }

  public PxVehicleTireData() : this(NativePINVOKE.new_PxVehicleTireData(), true) {
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public float mLatStiffX {
    set {
      NativePINVOKE.PxVehicleTireData_mLatStiffX_set(swigCPtr, value);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      float ret = NativePINVOKE.PxVehicleTireData_mLatStiffX_get(swigCPtr);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public float mLatStiffY {
    set {
      NativePINVOKE.PxVehicleTireData_mLatStiffY_set(swigCPtr, value);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      float ret = NativePINVOKE.PxVehicleTireData_mLatStiffY_get(swigCPtr);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public float mLongitudinalStiffnessPerUnitGravity {
    set {
      NativePINVOKE.PxVehicleTireData_mLongitudinalStiffnessPerUnitGravity_set(swigCPtr, value);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      float ret = NativePINVOKE.PxVehicleTireData_mLongitudinalStiffnessPerUnitGravity_get(swigCPtr);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public float mCamberStiffnessPerUnitGravity {
    set {
      NativePINVOKE.PxVehicleTireData_mCamberStiffnessPerUnitGravity_set(swigCPtr, value);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      float ret = NativePINVOKE.PxVehicleTireData_mCamberStiffnessPerUnitGravity_get(swigCPtr);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public uint mType {
    set {
      NativePINVOKE.PxVehicleTireData_mType_set(swigCPtr, value);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      uint ret = NativePINVOKE.PxVehicleTireData_mType_get(swigCPtr);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public float getRecipLongitudinalStiffnessPerUnitGravity() {
    float ret = NativePINVOKE.PxVehicleTireData_getRecipLongitudinalStiffnessPerUnitGravity(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public float getFrictionVsSlipGraphRecipx1Minusx0() {
    float ret = NativePINVOKE.PxVehicleTireData_getFrictionVsSlipGraphRecipx1Minusx0(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public float getFrictionVsSlipGraphRecipx2Minusx1() {
    float ret = NativePINVOKE.PxVehicleTireData_getFrictionVsSlipGraphRecipx2Minusx1(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

}

}
