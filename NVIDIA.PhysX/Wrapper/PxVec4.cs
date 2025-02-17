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
public partial struct PxVec4 {

  public float x,y,z,w;

  internal global::System.IntPtr swigCPtr {
    get { unsafe { fixed(PxVec4* p = &this) return (global::System.IntPtr)p; } }
  }

  internal PxVec4(global::System.IntPtr ptr, bool unused) {
      //this = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxVec4>(ptr);
      unsafe { this = *(PxVec4*)ptr; }
  }
    
  public PxVec4(float s) : this(NativePINVOKE.new_PxVec4__SWIG_0(s), true) { 
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve(); }
  public PxVec4(float x, float y, float z, float w) : this(NativePINVOKE.new_PxVec4__SWIG_1(x, y, z, w), true) { 
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve(); }
  public PxVec4( PxVec3  xyz, float w) : this(NativePINVOKE.new_PxVec4__SWIG_2( xyz.swigCPtr , w), true) { 
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve(); }
  public PxVec4( PxVec4  v) : this(NativePINVOKE.new_PxVec4__SWIG_3( v.swigCPtr ), true) { 
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve(); }
  public bool isZero() {
    bool ret = NativePINVOKE.PxVec4_isZero(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public bool isFinite() {
    bool ret = NativePINVOKE.PxVec4_isFinite(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public bool isNormalized() {
    bool ret = NativePINVOKE.PxVec4_isNormalized(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public float magnitudeSquared() {
    float ret = NativePINVOKE.PxVec4_magnitudeSquared(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public float magnitude() {
    float ret = NativePINVOKE.PxVec4_magnitude(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public float dot( PxVec4  v) {
    float ret = NativePINVOKE.PxVec4_dot(swigCPtr,  v.swigCPtr );
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public  PxVec4  getNormalized() {
        global::System.IntPtr ptr = NativePINVOKE.PxVec4_getNormalized(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
        //PxVec4 ret = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxVec4>(ptr);
        PxVec4 ret; unsafe { ret = *(PxVec4*)ptr; }
        return ret;
    }

  public float normalize() {
    float ret = NativePINVOKE.PxVec4_normalize(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public  PxVec4  multiply( PxVec4  a) {
        global::System.IntPtr ptr = NativePINVOKE.PxVec4_multiply(swigCPtr,  a.swigCPtr );
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
        //PxVec4 ret = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxVec4>(ptr);
        PxVec4 ret; unsafe { ret = *(PxVec4*)ptr; }
        return ret;
    }

  public  PxVec4  minimum( PxVec4  v) {
        global::System.IntPtr ptr = NativePINVOKE.PxVec4_minimum(swigCPtr,  v.swigCPtr );
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
        //PxVec4 ret = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxVec4>(ptr);
        PxVec4 ret; unsafe { ret = *(PxVec4*)ptr; }
        return ret;
    }

  public  PxVec4  maximum( PxVec4  v) {
        global::System.IntPtr ptr = NativePINVOKE.PxVec4_maximum(swigCPtr,  v.swigCPtr );
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
        //PxVec4 ret = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxVec4>(ptr);
        PxVec4 ret; unsafe { ret = *(PxVec4*)ptr; }
        return ret;
    }

  public  PxVec3  getXYZ() {
        global::System.IntPtr ptr = NativePINVOKE.PxVec4_getXYZ(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
        //PxVec3 ret = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxVec3>(ptr);
        PxVec3 ret; unsafe { ret = *(PxVec3*)ptr; }
        return ret;
    }

  public void setZero() {
    NativePINVOKE.PxVec4_setZero(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

}

}
