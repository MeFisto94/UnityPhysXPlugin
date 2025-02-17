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

public partial struct PxRigidBodyList {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwn;

  internal PxRigidBodyList(global::System.IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(PxRigidBodyList obj) {
    return obj.swigCPtr;
  }

  public uint count {
    get {
      uint ret = NativePINVOKE.PxRigidBodyList_count_get(swigCPtr);
      if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public PxRigidBody get(uint index) {
    global::System.IntPtr cPtr = NativePINVOKE.PxRigidBodyList_get(swigCPtr, index);
    PxRigidBody ret = (cPtr == global::System.IntPtr.Zero) ? null : new PxRigidBody(cPtr, false);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

}

}
