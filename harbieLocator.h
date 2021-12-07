//-
// ==========================================================================
// Copyright 2015 Autodesk, Inc.  All rights reserved.
// Use of this software is subject to the terms of the Autodesk license
// agreement provided at the time of installation or download, or which
// otherwise accompanies this software in either electronic or hard copy form.
// ==========================================================================
//+

////////////////////////////////////////////////////////////////////////
// DESCRIPTION:
//
//
// This plug-in demonstrates how to draw a simple mesh like foot Print in an
// easy way.
//
// This easy way is supported in Viewport 2.0.
// In Viewport 2.0, MUIDrawManager can used to draw simple UI elements in method
// addUIDrawables().
//
// For comparison, you can reference a Maya Developer Kit sample named
// rawharbieLocatorNode. In that sample, we draw the harbieLocator with
// OpenGL\DX in method rawharbieLocatorDrawOverride::draw().
//
// Note the method
//   harbieLocator::draw()
// is only called in legacy default viewport to draw foot Print.
// while the methods
//   harbieLocatorDrawOverride::prepareForDraw()
//   harbieLocatorDrawOverride::addUIDrawables()
// are only called in Viewport 2.0 to prepare and draw foot Print.
//
////////////////////////////////////////////////////////////////////////

#include <maya/M3dView.h>
#include <maya/MColor.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MDistance.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnCamera.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MPlug.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MString.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MTypeId.h>
#include <maya/MVector.h>
// Viewport 2.0 includes
#include <assert.h>
#include <maya/MDrawContext.h>
#include <maya/MDrawRegistry.h>
#include <maya/MEventMessage.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MPointArray.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MUintArray.h>
#include <maya/MUserData.h>

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

class harbieLocator : public MPxLocatorNode {
   public:
    // harbieLocator(harbieLocatorData& d)
    //	: data(d) {};
    harbieLocator();
    virtual ~harbieLocator();

    virtual MStatus compute(const MPlug& plug, MDataBlock& data);
    virtual void harbieLocator::postConstructor();
    virtual void draw(M3dView& view, const MDagPath& path,
                      M3dView::DisplayStyle style,
                      M3dView::DisplayStatus status);
    virtual MStatus setDependentsDirty(const MPlug& dirty_plug,
                                       MPlugArray& affected_plugs);

    virtual bool isBounded() const;
    virtual MBoundingBox boundingBox() const;

    static void* creator();
    static MStatus initialize();

    static MObject display;
    static MObject _size;
    static MObject _showCenter;
    static MObject _showOrientation;
    static MObject _centerScale;
    static MObject _distanceDisplay;
    static MObject _rotX;
    static MObject _rotY;
    static MObject _rotZ;
    static MObject _rot;
    static MObject _updateAttrs;

   public:
    static MTypeId id;
    static MString drawDbClassification;
    static MString drawRegistrantId;

   private:
    MObject _self;
    MMatrix _transformMatrix;

    // harbieLocatorData&      data;
};

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Viewport 2.0 override implementation
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class harbieLocatorData : public MUserData {
   private:
    MPoint getPointPositionDisplay(MPoint inPoint);

   public:
    void getFromDagPaths();
    harbieLocatorData() : MUserData(false){};  // don't delete after draw
    virtual ~harbieLocatorData(){};

    virtual void getListOfPoints(const MObject&);
    virtual void checkUpdateAttr(const MObject&);
    virtual void updateListOfPoints();
    virtual void getBB();
    virtual void getPlugs(const MObject&);

    bool updateAttrs = true;

    MColor fColor;
    std::vector<MPointArray> fLineList;
    std::vector<MPointArray> fTriangleList;
    MPointArray fullLineList;
    MUintArray fullListIndices;

    // to draw in front of camera
    double distanceDisplay;
    bool computeCameraAdjustment;
    MMatrix projectionMatrix;
    MMatrix current_camera_IM;
    MMatrix current_camera_IMI;
    MMatrix worldMatrix;
    MMatrix worldMatrixInverse;

    bool isOrtho;
    double nearClipZ;

    MBoundingBox theBoundingBox;
    MMatrix matPreRotate;

    int displayIndex = -1;
    float centerScale;
    bool showCenter;
    bool showOrientation;

    MDagPath objPath;
    MDagPath cameraPath;
};

class harbieLocatorDrawOverride : public MHWRender::MPxDrawOverride {
   public:
    static MHWRender::MPxDrawOverride* Creator(const MObject& obj) {
        return new harbieLocatorDrawOverride(obj);
    }

    virtual ~harbieLocatorDrawOverride();

    virtual MHWRender::DrawAPI supportedDrawAPIs() const;

    virtual bool isBounded(const MDagPath& objPath,
                           const MDagPath& cameraPath) const;

    virtual MBoundingBox boundingBox(const MDagPath& objPath,
                                     const MDagPath& cameraPath) const;

    virtual MUserData* prepareForDraw(
        const MDagPath& objPath, const MDagPath& cameraPath,
        const MHWRender::MFrameContext& frameContext, MUserData* oldData);

    virtual bool hasUIDrawables() const { return true; }

    virtual void addUIDrawables(const MDagPath& objPath,
                                MHWRender::MUIDrawManager& drawManager,
                                const MHWRender::MFrameContext& frameContext,
                                const MUserData* data);

    virtual bool traceCallSequence() const {
        // Return true if internal tracing is desired.
        return false;
    }
    virtual void handleTraceMessage(const MString& message) const {
        MGlobal::displayInfo("harbieLocatorDrawOverride: " + message);

        // Some simple custom message formatting.
        fprintf(stderr, "harbieLocatorDrawOverride: ");
        fprintf(stderr, message.asChar());
        fprintf(stderr, "\n");
    }

   private:
    harbieLocatorDrawOverride(const MObject& obj);
    static void OnModelEditorChanged(void* clientData);

    harbieLocator* fharbieLocator;
    MCallbackId fModelEditorChangedCbId;
};
