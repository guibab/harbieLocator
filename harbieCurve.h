#include <assert.h>
#include <maya/M3dView.h>
#include <maya/MColor.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MDistance.h>
#include <maya/MEulerRotation.h>
#include <maya/MEventMessage.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnNurbsCurveData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPointArray.h>
#include <maya/MPxNode.h>
#include <maya/MString.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MTypeId.h>
#include <maya/MUserData.h>
#include <maya/MVector.h>

#include <vector>

#define McheckErr(stat, msg)    \
    if (MS::kSuccess != stat) { \
        cerr << msg;            \
        return MS::kFailure;    \
    }

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Node implementation with standard viewport draw
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class harbieCurveData : public MUserData {
   public:
    harbieCurveData() : MUserData(false){};  // don't delete after draw
    virtual ~harbieCurveData(){};

    virtual void get(const MObject&, MMatrix);
    virtual void getPlugs(const MObject&);

    MColor fColor;
    std::vector<MPointArray> fLineList;
    MPointArray fullLineList;
    MMatrix matPreRotate;
    bool isOpen;
    int degree;
};

class harbieCurve : public MPxNode {
   public:
    // harbieCurve(harbieCurveData& d)
    //	: data(d) {};
    harbieCurve();
    virtual ~harbieCurve();
    static void* creator();

    virtual MStatus compute(const MPlug& plug, MDataBlock& data);
    // virtual void            harbieCurve::postConstructor();
    // virtual MStatus         setDependentsDirty(const MPlug& dirty_plug,
    // MPlugArray& affected_plugs);
    static MStatus initialize();

   public:
    static MTypeId id;
    static MObject display;
    static MObject _size;
    static MObject _showCenter;
    static MObject _showOrientation;
    static MObject _centerScale;
    static MObject _transX;
    static MObject _transY;
    static MObject _transZ;
    static MObject _trans;

    static MObject _scaleX;
    static MObject _scaleY;
    static MObject _scaleZ;
    static MObject _scale;

    static MObject _rotX;
    static MObject _rotY;
    static MObject _rotZ;
    static MObject _rot;

    static MObject outputCurve;

   private:
    MMatrix _transformMatrix;

    // harbieCurveData&      data;
};
