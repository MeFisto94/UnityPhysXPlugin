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

[global::System.Runtime.InteropServices.StructLayout(global::System.Runtime.InteropServices.LayoutKind.Sequential)]
public partial struct PxSpatialForce {

  public PxVec3 force,torque;

  internal global::System.IntPtr swigCPtr {
    get { unsafe { fixed(PxSpatialForce* p = &this) return (global::System.IntPtr)p; } }
  }

  internal PxSpatialForce(global::System.IntPtr ptr, bool unused) {
      //this = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxSpatialForce>(ptr);
      unsafe { this = *(PxSpatialForce*)ptr; }
  }
    
}

}
