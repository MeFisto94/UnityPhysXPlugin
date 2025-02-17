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

public partial class PxShape : PxBase {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;

  internal PxShape(global::System.IntPtr cPtr, bool cMemoryOwn) : base(NativePINVOKE.PxShape_SWIGUpcast(cPtr), cMemoryOwn) {
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(PxShape obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  internal static new PxShape getWrapper(global::System.IntPtr cPtr, bool cMemoryOwn) {
      var wrapper = WrapperCache.find(cPtr);
      if (!(wrapper is PxShape)) {
          wrapper = new PxShape(cPtr, cMemoryOwn);
          WrapperCache.add(cPtr, wrapper);
      }
      return wrapper as PxShape;
  }

  public uint getReferenceCount() {
    uint ret = NativePINVOKE.PxShape_getReferenceCount(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void acquireReference() {
    NativePINVOKE.PxShape_acquireReference(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public PxGeometryType getGeometryType() {
    PxGeometryType ret = (PxGeometryType)NativePINVOKE.PxShape_getGeometryType(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setGeometry(PxGeometry geometry) {
    NativePINVOKE.PxShape_setGeometry(swigCPtr, PxGeometry.getCPtr(geometry));
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public PxGeometryHolder getGeometry() {
    PxGeometryHolder ret = new PxGeometryHolder(NativePINVOKE.PxShape_getGeometry(swigCPtr), true);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public PxRigidActor getActor() {
    global::System.IntPtr cPtr = NativePINVOKE.PxShape_getActor(swigCPtr);
    PxRigidActor ret = (cPtr == global::System.IntPtr.Zero) ? null : PxRigidActor.getWrapper(cPtr, false);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setLocalPose( PxTransform  pose) {
    NativePINVOKE.PxShape_setLocalPose(swigCPtr,  pose.swigCPtr );
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public  PxTransform  getLocalPose() {
        global::System.IntPtr ptr = NativePINVOKE.PxShape_getLocalPose(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
        //PxTransform ret = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxTransform>(ptr);
        PxTransform ret; unsafe { ret = *(PxTransform*)ptr; }
        return ret;
    }

  public void setSimulationFilterData( PxFilterData  data) {
    NativePINVOKE.PxShape_setSimulationFilterData(swigCPtr,  data.swigCPtr );
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public  PxFilterData  getSimulationFilterData() {
        global::System.IntPtr ptr = NativePINVOKE.PxShape_getSimulationFilterData(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
        //PxFilterData ret = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxFilterData>(ptr);
        PxFilterData ret; unsafe { ret = *(PxFilterData*)ptr; }
        return ret;
    }

  public void setQueryFilterData( PxFilterData  data) {
    NativePINVOKE.PxShape_setQueryFilterData(swigCPtr,  data.swigCPtr );
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public  PxFilterData  getQueryFilterData() {
        global::System.IntPtr ptr = NativePINVOKE.PxShape_getQueryFilterData(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
        //PxFilterData ret = global::System.Runtime.InteropServices.Marshal.PtrToStructure<PxFilterData>(ptr);
        PxFilterData ret; unsafe { ret = *(PxFilterData*)ptr; }
        return ret;
    }

  public void setMaterials(PxMaterial[] materials, ushort materialCount) {
    NativePINVOKE.PxShape_setMaterials(swigCPtr, global::System.Array.ConvertAll(materials, x => PxMaterial.getCPtr(x)), materialCount);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public ushort getNbMaterials() {
    ushort ret = NativePINVOKE.PxShape_getNbMaterials(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public PxMaterial getMaterial(uint index) {
    global::System.IntPtr cPtr = NativePINVOKE.PxShape_getMaterial(swigCPtr, index);
    PxMaterial ret = (cPtr == global::System.IntPtr.Zero) ? null : PxMaterial.getWrapper(cPtr, false);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public PxMaterial getMaterialFromInternalFaceIndex(uint faceIndex) {
    global::System.IntPtr cPtr = NativePINVOKE.PxShape_getMaterialFromInternalFaceIndex(swigCPtr, faceIndex);
    PxMaterial ret = (cPtr == global::System.IntPtr.Zero) ? null : PxMaterial.getWrapper(cPtr, false);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setContactOffset(float contactOffset) {
    NativePINVOKE.PxShape_setContactOffset(swigCPtr, contactOffset);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public float getContactOffset() {
    float ret = NativePINVOKE.PxShape_getContactOffset(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setRestOffset(float restOffset) {
    NativePINVOKE.PxShape_setRestOffset(swigCPtr, restOffset);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public float getRestOffset() {
    float ret = NativePINVOKE.PxShape_getRestOffset(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setTorsionalPatchRadius(float radius) {
    NativePINVOKE.PxShape_setTorsionalPatchRadius(swigCPtr, radius);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public float getTorsionalPatchRadius() {
    float ret = NativePINVOKE.PxShape_getTorsionalPatchRadius(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setMinTorsionalPatchRadius(float radius) {
    NativePINVOKE.PxShape_setMinTorsionalPatchRadius(swigCPtr, radius);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public float getMinTorsionalPatchRadius() {
    float ret = NativePINVOKE.PxShape_getMinTorsionalPatchRadius(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setFlag(PxShapeFlag flag, bool value) {
    NativePINVOKE.PxShape_setFlag(swigCPtr, (int)flag, value);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public void setFlags(PxShapeFlag inFlags) {
    NativePINVOKE.PxShape_setFlags(swigCPtr, (int)inFlags);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public PxShapeFlag getFlags() {
    PxShapeFlag ret = (PxShapeFlag)NativePINVOKE.PxShape_getFlags(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public bool isExclusive() {
    bool ret = NativePINVOKE.PxShape_isExclusive(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void setName(string name) {
    NativePINVOKE.PxShape_setName(swigCPtr, name);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
  }

  public string getName() {
    string ret = NativePINVOKE.PxShape_getName(swigCPtr);
    if (NativePINVOKE.SWIGPendingException.Pending) throw NativePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

}

}
