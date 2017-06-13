#include "harbieCurve.h"

#include "shapesDefinitionCurve.h"

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Node implementation with standard viewport draw
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

MObject harbieCurve::display;

MObject harbieCurve::_size;
MObject harbieCurve::_rotX;
MObject harbieCurve::_rotY;
MObject harbieCurve::_rotZ;
MObject harbieCurve::_rot;

MObject harbieCurve::_transX;
MObject harbieCurve::_transY;
MObject harbieCurve::_transZ;
MObject harbieCurve::_trans;

MObject harbieCurve::_scaleX;
MObject harbieCurve::_scaleY;
MObject harbieCurve::_scaleZ;
MObject harbieCurve::_scale;

MObject harbieCurve::outputCurve;

MTypeId harbieCurve::id(0x001226F5);

harbieCurve::harbieCurve() {}
harbieCurve::~harbieCurve() {}
/*

void harbieCurve::postConstructor() {
        MObject self = thisMObject();
        MFnDependencyNode fn_node(self);
        fn_node.setName("harbieCurveShape#");

        _self = self;
        _update_attrs = true;
        MMatrix _transformMatrix;
}
*/
/*
MStatus harbieCurve::setDependentsDirty(const MPlug& dirty_plug, MPlugArray&
affected_plugs) { MString plug_name_MString = dirty_plug.partialName();
        std::string plug_name = plug_name_MString.asChar();
        if( (plug_name == "lpx")|| (plug_name == "lpy")|| (plug_name == "lpz")
){ _update_attrs = true;
        }
        if ((plug_name == "lsx") || (plug_name == "lsy") || (plug_name ==
"lsz")) { _update_attrs = true;
        }
        if ((dirty_plug == _rot) || (dirty_plug == _rotX) || (dirty_plug ==
_rotY) || (dirty_plug == _rotZ)) { _update_attrs = true;
        }
        return MS::kSuccess;
}
*/

void harbieCurveData::getPlugs(const MObject& node) {
    MStatus status;
    double size = MPlug(node, harbieCurve::_size).asDouble();
    double tx = MPlug(node, harbieCurve::_transX).asDouble();
    double ty = MPlug(node, harbieCurve::_transY).asDouble();
    double tz = MPlug(node, harbieCurve::_transZ).asDouble();

    double sx = MPlug(node, harbieCurve::_scaleX).asDouble();
    double sy = MPlug(node, harbieCurve::_scaleY).asDouble();
    double sz = MPlug(node, harbieCurve::_scaleZ).asDouble();

    double rx = MPlug(node, harbieCurve::_rotX).asDouble();
    double ry = MPlug(node, harbieCurve::_rotY).asDouble();
    double rz = MPlug(node, harbieCurve::_rotZ).asDouble();
    MEulerRotation eulerRot(rx, ry, rz);

    this->matPreRotate = eulerRot.asMatrix();
    this->matPreRotate.matrix[0][0] *= sx * size;
    this->matPreRotate.matrix[0][1] *= sx * size;
    this->matPreRotate.matrix[0][2] *= sx * size;
    this->matPreRotate.matrix[1][0] *= sy * size;
    this->matPreRotate.matrix[1][1] *= sy * size;
    this->matPreRotate.matrix[1][2] *= sy * size;
    this->matPreRotate.matrix[2][0] *= sz * size;
    this->matPreRotate.matrix[2][1] *= sz * size;
    this->matPreRotate.matrix[2][2] *= sz * size;
    this->matPreRotate.matrix[3][0] = tx;
    this->matPreRotate.matrix[3][1] = ty;
    this->matPreRotate.matrix[3][2] = tz;
}
void harbieCurveData::get(const MObject& node, MMatrix matPreRotate) {
    MStatus status;

    // MMatrix matPreRotate = node._transformMatrix;
    MPlug displayType(node, harbieCurve::display);
    int displayIndex = displayType.asInt();

    this->fLineList.clear();
    this->fLineList.resize(1);
    MPointArray& linesPoints = this->fLineList[0];

    if (displayIndex == 0) {  // arrow
        for (int i = 0; i < arrowCount; i++)
            linesPoints.append(MPoint(listLinesArrow[i][0],
                                      listLinesArrow[i][1],
                                      listLinesArrow[i][2]) *
                               matPreRotate);
        this->isOpen = arrowisOpen;
        this->degree = arrowDegree;
    }

    else if (displayIndex == 1) {  // bone
        for (int i = 0; i < boneCount; i++)
            linesPoints.append(MPoint(listLinesBone[i][0], listLinesBone[i][1],
                                      listLinesBone[i][2]) *
                               matPreRotate);
        this->isOpen = boneisOpen;
        this->degree = boneDegree;
    }

    else if (displayIndex == 2) {  // circle
        for (int i = 0; i < circleCount; i++)
            linesPoints.append(MPoint(listLinesCircle[i][0],
                                      listLinesCircle[i][1],
                                      listLinesCircle[i][2]) *
                               matPreRotate);
        this->isOpen = circleisOpen;
        this->degree = circleDegree;
    }

    else if (displayIndex == 3) {  // compass
        for (int i = 0; i < compassCount; i++)
            linesPoints.append(MPoint(listLinesCompass[i][0],
                                      listLinesCompass[i][1],
                                      listLinesCompass[i][2]) *
                               matPreRotate);
        this->isOpen = compassisOpen;
        this->degree = compassDegree;
    }

    else if (displayIndex == 4) {  // cross
        for (int i = 0; i < crossCount; i++)
            linesPoints.append(MPoint(listLinesCross[i][0],
                                      listLinesCross[i][1],
                                      listLinesCross[i][2]) *
                               matPreRotate);
        this->isOpen = crossisOpen;
        this->degree = crossDegree;
    }

    else if (displayIndex == 5) {  // crossArrow
        for (int i = 0; i < crossArrowCount; i++)
            linesPoints.append(MPoint(listLinesCrossarrow[i][0],
                                      listLinesCrossarrow[i][1],
                                      listLinesCrossarrow[i][2]) *
                               matPreRotate);
        this->isOpen = crossArrowisOpen;
        this->degree = crossArrowDegree;
    }

    else if (displayIndex == 6) {  // cube
        for (int i = 0; i < cubeCount; i++)
            linesPoints.append(MPoint(listLinesCube[i][0], listLinesCube[i][1],
                                      listLinesCube[i][2]) *
                               matPreRotate);
        this->isOpen = cubeisOpen;
        this->degree = cubeDegree;
    }

    else if (displayIndex == 7) {  // cubeWithPeak
        for (int i = 0; i < cubeWithPeakCount; i++)
            linesPoints.append(MPoint(listLinesCubewithpeak[i][0],
                                      listLinesCubewithpeak[i][1],
                                      listLinesCubewithpeak[i][2]) *
                               matPreRotate);
        this->isOpen = cubeWithPeakisOpen;
        this->degree = cubeWithPeakDegree;
    }

    else if (displayIndex == 8) {  // cylinder
        for (int i = 0; i < cylinderCount; i++)
            linesPoints.append(MPoint(listLinesCylinder[i][0],
                                      listLinesCylinder[i][1],
                                      listLinesCylinder[i][2]) *
                               matPreRotate);
        this->isOpen = cylinderisOpen;
        this->degree = cylinderDegree;
    }

    else if (displayIndex == 9) {  // diamond
        for (int i = 0; i < diamondCount; i++)
            linesPoints.append(MPoint(listLinesDiamond[i][0],
                                      listLinesDiamond[i][1],
                                      listLinesDiamond[i][2]) *
                               matPreRotate);
        this->isOpen = diamondisOpen;
        this->degree = diamondDegree;
    }

    else if (displayIndex == 10) {  // flower
        for (int i = 0; i < flowerCount; i++)
            linesPoints.append(MPoint(listLinesFlower[i][0],
                                      listLinesFlower[i][1],
                                      listLinesFlower[i][2]) *
                               matPreRotate);
        this->isOpen = flowerisOpen;
        this->degree = flowerDegree;
    }

    else if (displayIndex == 11) {  // jaw
        for (int i = 0; i < jawCount; i++)
            linesPoints.append(MPoint(listLinesJaw[i][0], listLinesJaw[i][1],
                                      listLinesJaw[i][2]) *
                               matPreRotate);
        this->isOpen = jawisOpen;
        this->degree = jawDegree;
    }

    else if (displayIndex == 12) {  // null
        for (int i = 0; i < nullCount; i++)
            linesPoints.append(MPoint(listLinesNull[i][0], listLinesNull[i][1],
                                      listLinesNull[i][2]) *
                               matPreRotate);
        this->isOpen = nullisOpen;
        this->degree = nullDegree;
    }

    else if (displayIndex == 13) {  // pyramid
        for (int i = 0; i < pyramidCount; i++)
            linesPoints.append(MPoint(listLinesPyramid[i][0],
                                      listLinesPyramid[i][1],
                                      listLinesPyramid[i][2]) *
                               matPreRotate);
        this->isOpen = pyramidisOpen;
        this->degree = pyramidDegree;
    }

    else if (displayIndex == 14) {  // sphere
        for (int i = 0; i < sphereCount; i++)
            linesPoints.append(MPoint(listLinesSphere[i][0],
                                      listLinesSphere[i][1],
                                      listLinesSphere[i][2]) *
                               matPreRotate);
        this->isOpen = sphereisOpen;
        this->degree = sphereDegree;
    }

    else if (displayIndex == 15) {  // spine
        for (int i = 0; i < spineCount; i++)
            linesPoints.append(MPoint(listLinesSpine[i][0],
                                      listLinesSpine[i][1],
                                      listLinesSpine[i][2]) *
                               matPreRotate);
        this->isOpen = spineisOpen;
        this->degree = spineDegree;
    }

    else if (displayIndex == 16) {  // square
        for (int i = 0; i < squareCount; i++)
            linesPoints.append(MPoint(listLinesSquare[i][0],
                                      listLinesSquare[i][1],
                                      listLinesSquare[i][2]) *
                               matPreRotate);
        this->isOpen = squareisOpen;
        this->degree = squareDegree;
    }
}

MStatus harbieCurve::compute(const MPlug& plug, MDataBlock& data) {
    MStatus stat;
    MObject thisNode = thisMObject();
    MFnDependencyNode fnThisNode(thisNode);

    harbieCurveData curveData;
    curveData.getPlugs(thisNode);
    curveData.get(thisNode, curveData.matPreRotate);
    // data.get(_self, _transformMatrix);

    // Draw the outline of the foot
    //
    MPointArray listOfLinesPoints;
    listOfLinesPoints = curveData.fLineList[0];
    int nbLines = curveData.fLineList.size();

    MDataHandle outputCurve_DataHandle =
        data.outputValue(harbieCurve::outputCurve);
    MFnNurbsCurveData dataCreator;
    MObject createdCurve = dataCreator.create(&stat);
    MPlug curvePlug = fnThisNode.findPlug(outputCurve, &stat);
    MPlugArray theConnectedCurveArray;
    MPlug theConnectedCurve;
    curvePlug.connectedTo(theConnectedCurveArray, 0, 1, &stat);
    MPlug worldMatrixCurvePlug;
    if (theConnectedCurveArray.length() >
        0) {  // if connected to a curve in output
        theConnectedCurve = theConnectedCurveArray[0];
        // cout << "curve " << theConnectedCurve.name().asChar() << endl;
        MObject objCurve = theConnectedCurve.node();
        MFnDagNode objCurveDagFn(objCurve);
        worldMatrixCurvePlug = objCurveDagFn.findPlug("worldMatrix");
        worldMatrixCurvePlug = worldMatrixCurvePlug.elementByLogicalIndex(0);
    } else {
        cout << "curve not found" << endl;
    }

    MObject curveMatrixObj;
    worldMatrixCurvePlug.getValue(curveMatrixObj);
    MFnMatrixData fndMatrixCurv(curveMatrixObj);
    MMatrix currentOuputCurveMat = fndMatrixCurv.matrix();
    MFnNurbsCurve curveFn;
    MDoubleArray knots;

    int degree_value = curveData.degree;
    int cvsLength = listOfLinesPoints.length();
    if (!curveData.isOpen) {
        listOfLinesPoints.append(listOfLinesPoints[0]);
        if (degree_value == 3) {
            listOfLinesPoints.append(listOfLinesPoints[1]);
            listOfLinesPoints.append(listOfLinesPoints[2]);
            knots.append(-2);
            knots.append(-1.);
            for (int i = 0; (i < cvsLength + 3); i++) {
                knots.append((double)i);
            }
        } else {
            knots.append(-1.);
            for (int i = 0; (i < cvsLength); i++) {
                knots.append((double)i);
            }
        }

    } else {
        // int cvsLength = listOfLinesPoints.length();
        if (degree_value == 3) {
            knots.append((double)0.);
            knots.append((double)0.);
            for (int i = 0; (i < (cvsLength - 2)); i++) {
                knots.append((double)i);
            }
            double lastVal = listOfLinesPoints.length() - 3.;
            knots.append(lastVal);
            knots.append(lastVal);
        } else {
            for (int i = 0; (i < cvsLength); i++) {
                knots.append((double)i);
            }
        }
    }
    if (curveData.isOpen)
        curveFn.create(listOfLinesPoints, knots, degree_value,
                       MFnNurbsCurve::kOpen, false, false, createdCurve, &stat);
    else
        curveFn.create(listOfLinesPoints, knots, degree_value,
                       MFnNurbsCurve::kPeriodic, false, false, createdCurve,
                       &stat);

    outputCurve_DataHandle.setMObject(createdCurve);
    outputCurve_DataHandle.asNurbsCurve();
    data.setClean(plug);

    return MS::kSuccess;
}

void* harbieCurve::creator() { return new harbieCurve(); }

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Plugin initialize
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

MStatus harbieCurve::initialize() {
    MFnUnitAttribute unitFn;
    MFnNumericAttribute nAttr;
    MFnEnumAttribute enumAttr;
    MFnTypedAttribute typedAttr;

    MStatus stat;

    _transX =
        nAttr.create("localTranslateX", "ltx", MFnNumericData::kDouble, 0.0);
    _transY =
        nAttr.create("localTranslateY", "lty", MFnNumericData::kDouble, 0.0);
    _transZ =
        nAttr.create("localTranslateZ", "ltz", MFnNumericData::kDouble, 0.0);
    _trans = nAttr.create("localTranslate", "lt", _transX, _transY, _transZ);
    nAttr.setDefault(0., 0., 0.);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setWritable(true);
    nAttr.setReadable(true);
    CHECK_MSTATUS(addAttribute(_trans));

    _scaleX = nAttr.create("localScaleX", "lsx", MFnNumericData::kDouble, 1.0);
    _scaleY = nAttr.create("localScaleY", "lsy", MFnNumericData::kDouble, 1.0);
    _scaleZ = nAttr.create("localScaleZ", "lsz", MFnNumericData::kDouble, 1.0);
    _scale = nAttr.create("localScale", "ls", _scaleX, _scaleY, _scaleZ);
    nAttr.setDefault(1., 1., 1.);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setWritable(true);
    nAttr.setReadable(true);
    CHECK_MSTATUS(addAttribute(_scale));

    _rotX = unitFn.create("localRotateX", "lrx", MFnUnitAttribute::kAngle, 0.0);
    _rotY = unitFn.create("localRotateY", "lry", MFnUnitAttribute::kAngle, 0.0);
    _rotZ = unitFn.create("localRotateZ", "lrz", MFnUnitAttribute::kAngle, 0.0);
    _rot = nAttr.create("localRotate", "lr", _rotX, _rotY, _rotZ);

    unitFn.setStorable(true);
    nAttr.setDefault(0., 0., 0.);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setWritable(true);
    nAttr.setReadable(true);
    CHECK_MSTATUS(addAttribute(_rot));

    _size = nAttr.create("size", "sz", MFnNumericData::kDouble, 1.0);
    nAttr.setDefault(1.);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setWritable(true);
    nAttr.setReadable(true);
    CHECK_MSTATUS(addAttribute(_size));

    // the type of deformation
    display = enumAttr.create("display", "display", 0);
    CHECK_MSTATUS(enumAttr.addField("Arrow", 0));
    CHECK_MSTATUS(enumAttr.addField("Bone", 1));
    CHECK_MSTATUS(enumAttr.addField("Circle", 2));
    CHECK_MSTATUS(enumAttr.addField("Compass", 3));
    CHECK_MSTATUS(enumAttr.addField("Cross", 4));
    CHECK_MSTATUS(enumAttr.addField("CrossArrow", 5));
    CHECK_MSTATUS(enumAttr.addField("Cube", 6));
    CHECK_MSTATUS(enumAttr.addField("CubeWithPeak", 7));
    CHECK_MSTATUS(enumAttr.addField("Cylinder", 8));
    CHECK_MSTATUS(enumAttr.addField("Diamond", 9));
    CHECK_MSTATUS(enumAttr.addField("Flower", 10));
    CHECK_MSTATUS(enumAttr.addField("Jaw", 11));
    CHECK_MSTATUS(enumAttr.addField("Null", 12));
    CHECK_MSTATUS(enumAttr.addField("Pyramid", 13));
    CHECK_MSTATUS(enumAttr.addField("Sphere", 14));
    CHECK_MSTATUS(enumAttr.addField("Spine", 15));
    CHECK_MSTATUS(enumAttr.addField("Square", 16));

    CHECK_MSTATUS(enumAttr.setStorable(true));
    CHECK_MSTATUS(enumAttr.setKeyable(true));
    CHECK_MSTATUS(enumAttr.setReadable(true));
    CHECK_MSTATUS(enumAttr.setWritable(true));
    CHECK_MSTATUS(enumAttr.setCached(false));

    CHECK_MSTATUS(addAttribute(display));

    outputCurve = typedAttr.create("outputCurve", "oc", MFnData::kNurbsCurve);
    typedAttr.setStorable(false);
    typedAttr.setKeyable(false);
    typedAttr.setWritable(false);
    typedAttr.setReadable(true);

    stat = addAttribute(outputCurve);
    // check for error
    if (stat != MS::kSuccess) {
        stat.perror("Unable to attach outputCurve attribute to the node");
        return stat;
    }
    stat = attributeAffects(harbieCurve::display, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_size, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_trans, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_transX, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_transY, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_transZ, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_rot, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_rotX, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_rotY, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_rotZ, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_scale, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_scaleX, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_scaleY, harbieCurve::outputCurve);
    stat = attributeAffects(harbieCurve::_scaleZ, harbieCurve::outputCurve);

    return MS::kSuccess;
}
