#include "harbieLocator.h"

#include "shapesDefinition.h"

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Node implementation with standard viewport draw
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
MTypeId harbieLocator::id(0x001226F4);

MObject harbieLocator::display;
MObject harbieLocator::_size;
MObject harbieLocator::_showCenter;
MObject harbieLocator::_showOrientation;
MObject harbieLocator::_centerScale;
MObject harbieLocator::_distanceDisplay;

MObject harbieLocator::_rotX;
MObject harbieLocator::_rotY;
MObject harbieLocator::_rotZ;
MObject harbieLocator::_rot;

MObject harbieLocator::_updateAttrs;

MString harbieLocator::drawDbClassification("drawdb/geometry/harbieLocator");
MString harbieLocator::drawRegistrantId("harbieLocatorNodePlugin");

harbieLocator::harbieLocator()
//:_update_attrs(true)
{}

harbieLocator::~harbieLocator() {}

void harbieLocator::postConstructor() {
    MStatus status;
    MObject self = thisMObject();
    MFnDependencyNode fn_node(self);

    MString iconName("harbieLocator.png");
    status = fn_node.setIcon(iconName);
    if (MS::kSuccess != status) {
        MGlobal::displayError(
            "nodeIcon: the filename specified by the -icon flag could not be "
            "opened");
    }

    fn_node.setName("harbieLocatorShape#");

    _self = self;
    //_update_attrs = true;
    // MMatrix _transformMatrix;
}

MStatus harbieLocator::setDependentsDirty(const MPlug& dirty_plug,
                                          MPlugArray& affected_plugs) {
    MString plug_name_MString = dirty_plug.partialName();
    std::string plug_name = plug_name_MString.asChar();
    bool doUpdateAttr = false;
    if ((plug_name == "lpx") || (plug_name == "lpy") || (plug_name == "lpz")) {
        doUpdateAttr = true;
    }
    if ((plug_name == "lsx") || (plug_name == "lsy") || (plug_name == "lsz")) {
        doUpdateAttr = true;
    }
    if ((dirty_plug == _rot) || (dirty_plug == _rotX) ||
        (dirty_plug == _rotY) || (dirty_plug == _rotZ)) {
        doUpdateAttr = true;
    }
    if ((dirty_plug == display) || (dirty_plug == _size) ||
        (dirty_plug == _showCenter) || (dirty_plug == _showOrientation) ||
        (dirty_plug == _centerScale) || (dirty_plug == _distanceDisplay)) {
        doUpdateAttr = true;
    }
    // MPlug(_updateAtts
    if (doUpdateAttr) {
        MPlug updateAttrsPlug(_self, _updateAttrs);
        updateAttrsPlug.setBool(true);
    }

    return MS::kSuccess;
}

void harbieLocatorData::checkUpdateAttr(const MObject& node) {
    MPlug updateAttrsPlug(node, harbieLocator::_updateAttrs);
    bool updateAttrsVal = updateAttrsPlug.asBool();
    if (updateAttrsVal) {
        this->updateAttrs = true;
    }
    // this->updateAttrs |= updateAttrsVal;
    // this->updateAttrs = true;
}

void harbieLocatorData::getPlugs(const MObject& node) {
    MStatus status;
    double size = MPlug(node, harbieLocator::_size).asDouble();
    float tx = MPlug(node, harbieLocator::localPositionX).asFloat();
    float ty = MPlug(node, harbieLocator::localPositionY).asFloat();
    float tz = MPlug(node, harbieLocator::localPositionZ).asFloat();

    float sx = MPlug(node, harbieLocator::localScaleX).asFloat();
    float sy = MPlug(node, harbieLocator::localScaleY).asFloat();
    float sz = MPlug(node, harbieLocator::localScaleZ).asFloat();

    float rx = MPlug(node, harbieLocator::_rotX).asFloat();
    float ry = MPlug(node, harbieLocator::_rotY).asFloat();
    float rz = MPlug(node, harbieLocator::_rotZ).asFloat();

    this->displayIndex = MPlug(node, harbieLocator::display).asInt();
    this->centerScale = MPlug(node, harbieLocator::_centerScale).asFloat();
    this->showCenter = MPlug(node, harbieLocator::_showCenter).asBool();
    this->showOrientation =
        MPlug(node, harbieLocator::_showOrientation).asBool();

    this->distanceDisplay =
        MPlug(node, harbieLocator::_distanceDisplay).asDouble();
    this->computeCameraAdjustment = this->distanceDisplay > 0;

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
void harbieLocatorData::getListOfPoints(const MObject& node) {
    MStatus status;
    int sizeToresize = 1;
    if (showCenter) sizeToresize++;
    if (showOrientation) sizeToresize++;

    this->fLineList.clear();
    this->fTriangleList.clear();

    this->fLineList.resize(sizeToresize);
    MPointArray& linesPoints = this->fLineList[0];
    linesPoints.clear();
    int currentInd = 0;
    if (showCenter) {
        currentInd++;
        MPointArray& linesPointsCenter = this->fLineList[currentInd];
        linesPointsCenter.clear();
        for (int i = 0; i < nullCount; i++) {
            MPoint pt = MPoint(listLinesNull[i][0] * centerScale * 0.5,
                               listLinesNull[i][1] * centerScale * 0.5,
                               listLinesNull[i][2] * centerScale * 0.5);
            linesPointsCenter.append(pt);
        }
    }
    if (showOrientation) {
        currentInd++;
        MPointArray& linesPointsCenterOrientation = this->fLineList[currentInd];
        linesPointsCenterOrientation.clear();
        for (int i = 0; i < lookAtCount; i++) {
            MPoint pt = MPoint(listLinesLookat[i][0] * centerScale,
                               listLinesLookat[i][1] * centerScale,
                               listLinesLookat[i][2] * centerScale);
            linesPointsCenterOrientation.append(pt);
        }
    }

    if (displayIndex == 0) {  // arrow
        for (int i = 0; i < arrowCount; i++) {
            MPoint pt = MPoint(listLinesArrow[i][0], listLinesArrow[i][1],
                               listLinesArrow[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 1) {  // bone
        for (int i = 0; i < boneCount; i++) {
            MPoint pt = MPoint(listLinesBone[i][0], listLinesBone[i][1],
                               listLinesBone[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 2) {  // circle
        for (int i = 0; i < circleCount; i++) {
            MPoint pt = MPoint(listLinesCircle[i][0], listLinesCircle[i][1],
                               listLinesCircle[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 3) {  // compass
        for (int i = 0; i < compassCount; i++) {
            MPoint pt = MPoint(listLinesCompass[i][0], listLinesCompass[i][1],
                               listLinesCompass[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 4) {  // cross
        for (int i = 0; i < crossCount; i++) {
            MPoint pt = MPoint(listLinesCross[i][0], listLinesCross[i][1],
                               listLinesCross[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 5) {  // crossArrow
        for (int i = 0; i < crossArrowCount; i++) {
            MPoint pt =
                MPoint(listLinesCrossarrow[i][0], listLinesCrossarrow[i][1],
                       listLinesCrossarrow[i][2]) *
                matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 6) {  // cube
        for (int i = 0; i < cubeCount; i++) {
            MPoint pt = MPoint(listLinesCube[i][0], listLinesCube[i][1],
                               listLinesCube[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 7) {  // cubeWithPeak
        for (int i = 0; i < cubeWithPeakCount; i++) {
            MPoint pt =
                MPoint(listLinesCubewithpeak[i][0], listLinesCubewithpeak[i][1],
                       listLinesCubewithpeak[i][2]) *
                matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 8) {  // cylinder
        for (int i = 0; i < cylinderCount; i++) {
            MPoint pt = MPoint(listLinesCylinder[i][0], listLinesCylinder[i][1],
                               listLinesCylinder[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 9) {  // diamond
        for (int i = 0; i < diamondCount; i++) {
            MPoint pt = MPoint(listLinesDiamond[i][0], listLinesDiamond[i][1],
                               listLinesDiamond[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 10) {  // flower
        for (int i = 0; i < flowerCount; i++) {
            MPoint pt = MPoint(listLinesFlower[i][0], listLinesFlower[i][1],
                               listLinesFlower[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 11) {  // jaw
        for (int i = 0; i < jawCount; i++) {
            MPoint pt = MPoint(listLinesJaw[i][0], listLinesJaw[i][1],
                               listLinesJaw[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 12) {  // null
        for (int i = 0; i < nullCount; i++) {
            MPoint pt = MPoint(listLinesNull[i][0], listLinesNull[i][1],
                               listLinesNull[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 13) {  // pyramid
        for (int i = 0; i < pyramidCount; i++) {
            MPoint pt = MPoint(listLinesPyramid[i][0], listLinesPyramid[i][1],
                               listLinesPyramid[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 14) {  // sphere
        for (int i = 0; i < sphereCount; i++) {
            MPoint pt = MPoint(listLinesSphere[i][0], listLinesSphere[i][1],
                               listLinesSphere[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 15) {  // spine
        for (int i = 0; i < spineCount; i++) {
            MPoint pt = MPoint(listLinesSpine[i][0], listLinesSpine[i][1],
                               listLinesSpine[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }

    else if (displayIndex == 16) {  // square
        for (int i = 0; i < squareCount; i++) {
            MPoint pt = MPoint(listLinesSquare[i][0], listLinesSquare[i][1],
                               listLinesSquare[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    } else if (displayIndex == 17) {  // lookAt
        for (int i = 0; i < lookAtCount; i++) {
            MPoint pt = MPoint(listLinesLookat[i][0], listLinesLookat[i][1],
                               listLinesLookat[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    } else if (displayIndex == 18) {  // bended arrow
        for (int i = 0; i < bendedArrowCount; i++) {
            MPoint pt =
                MPoint(listLinesBendedarrow[i][0], listLinesBendedarrow[i][1],
                       listLinesBendedarrow[i][2]) *
                matPreRotate;
            linesPoints.append(pt);
        }
    } else if (displayIndex == 19) {  // rotate arrow
        for (int i = 0; i < rotateArrowCount; i++) {
            MPoint pt =
                MPoint(listLinesRotatearrow[i][0], listLinesRotatearrow[i][1],
                       listLinesRotatearrow[i][2]) *
                matPreRotate;
            linesPoints.append(pt);
        }
    } else if (displayIndex == 20) {  // gear
        for (int i = 0; i < gearCount; i++) {
            MPoint pt = MPoint(listLinesGear[i][0], listLinesGear[i][1],
                               listLinesGear[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    } else if (displayIndex == 21) {  // lung
        for (int i = 0; i < lungsCount; i++) {
            MPoint pt = MPoint(listLinesLungs[i][0], listLinesLungs[i][1],
                               listLinesLungs[i][2]) *
                        matPreRotate;
            linesPoints.append(pt);
        }
    }
    // linesPoints
    updateListOfPoints();
}
void harbieLocatorData::updateListOfPoints() {
    // could be better
    this->fullLineList.clear();
    this->fullListIndices.clear();
    int nbLines = this->fLineList.size();

    for (int i = 0; i < nbLines; ++i) {
        int nbVertices = this->fLineList[i].length();
        int startPt = this->fullLineList.length();

        for (int j = 0; j < nbVertices; ++j) {
            MPoint pt = this->fLineList[i][j];
            if (computeCameraAdjustment) pt = this->getPointPositionDisplay(pt);
            this->fullLineList.append(pt);
        }
        for (int j = 1; j < nbVertices; ++j) {
            this->fullListIndices.append(startPt + j - 1);
            this->fullListIndices.append(startPt + j);
        }
    }
}

MPoint harbieLocatorData::getPointPositionDisplay(MPoint inPoint) {
    MPoint worldPoint = inPoint * this->worldMatrix;
    MPoint resPoint = worldPoint * this->current_camera_IMI;

    if (!this->isOrtho) {
        MVector resVec = MVector(resPoint);
        double newLength = resVec.length() - this->distanceDisplay;
        if (newLength < this->nearClipZ) {
            newLength = 1.1 * this->nearClipZ;
        }
        resVec.normalize();
        resPoint = MPoint(newLength * resVec);
    } else {
        /*
        MPoint matPoint = MPoint(0,0,0) * this->current_camera_IM;
        MVector vec(worldPoint - matPoint);
        double dst = vec.length() - this->distanceDisplay;
        MPoint ToReturn = dst * matPoint + matPoint;
        return ToReturn * this->worldMatrixInverse;
        */

        resPoint.z += this->distanceDisplay;
        if ((resPoint.z * -1) < this->nearClipZ) {
            resPoint.z = -1.1 * this->nearClipZ;
        }
    }

    resPoint = resPoint * this->current_camera_IM;
    resPoint = resPoint * this->worldMatrixInverse;
    return resPoint;
}
void harbieLocatorData::getFromDagPaths() {
    if (this->computeCameraAdjustment) {
        this->current_camera_IM = this->cameraPath.inclusiveMatrix();
        this->current_camera_IMI = this->cameraPath.inclusiveMatrixInverse();

        MFnCamera real_camera(cameraPath);
        this->isOrtho = real_camera.isOrtho();
        this->nearClipZ = real_camera.nearClippingPlane();
        this->worldMatrix = this->objPath.inclusiveMatrix();
        this->worldMatrixInverse = this->objPath.inclusiveMatrixInverse();
    }
}
void harbieLocatorData::getBB() {
    MStatus status;
    // MMatrix matPreRotate = node._transformMatrix;
    if (displayIndex == 0) {  // arrow
        this->theBoundingBox =
            MBoundingBox(MPoint(arrowBB[0][0], arrowBB[0][1], arrowBB[0][2]),
                         MPoint(arrowBB[1][0], arrowBB[1][1], arrowBB[1][2]));
    }

    else if (displayIndex == 1) {  // bone
        this->theBoundingBox =
            MBoundingBox(MPoint(boneBB[0][0], boneBB[0][1], boneBB[0][2]),
                         MPoint(boneBB[1][0], boneBB[1][1], boneBB[1][2]));
    }

    else if (displayIndex == 2) {  // circle
        this->theBoundingBox = MBoundingBox(
            MPoint(circleBB[0][0], circleBB[0][1], circleBB[0][2]),
            MPoint(circleBB[1][0], circleBB[1][1], circleBB[1][2]));
    }

    else if (displayIndex == 3) {  // compass
        this->theBoundingBox = MBoundingBox(
            MPoint(compassBB[0][0], compassBB[0][1], compassBB[0][2]),
            MPoint(compassBB[1][0], compassBB[1][1], compassBB[1][2]));
    }

    else if (displayIndex == 4) {  // cross
        this->theBoundingBox =
            MBoundingBox(MPoint(crossBB[0][0], crossBB[0][1], crossBB[0][2]),
                         MPoint(crossBB[1][0], crossBB[1][1], crossBB[1][2]));
    }

    else if (displayIndex == 5) {  // crossArrow
        this->theBoundingBox = MBoundingBox(
            MPoint(crossArrowBB[0][0], crossArrowBB[0][1], crossArrowBB[0][2]),
            MPoint(crossArrowBB[1][0], crossArrowBB[1][1], crossArrowBB[1][2]));
    }

    else if (displayIndex == 6) {  // cube
        this->theBoundingBox =
            MBoundingBox(MPoint(cubeBB[0][0], cubeBB[0][1], cubeBB[0][2]),
                         MPoint(cubeBB[1][0], cubeBB[1][1], cubeBB[1][2]));
    }

    else if (displayIndex == 7) {  // cubeWithPeak
        this->theBoundingBox =
            MBoundingBox(MPoint(cubeWithPeakBB[0][0], cubeWithPeakBB[0][1],
                                cubeWithPeakBB[0][2]),
                         MPoint(cubeWithPeakBB[1][0], cubeWithPeakBB[1][1],
                                cubeWithPeakBB[1][2]));
    }

    else if (displayIndex == 8) {  // cylinder
        this->theBoundingBox = MBoundingBox(
            MPoint(cylinderBB[0][0], cylinderBB[0][1], cylinderBB[0][2]),
            MPoint(cylinderBB[1][0], cylinderBB[1][1], cylinderBB[1][2]));
    }

    else if (displayIndex == 9) {  // diamond
        this->theBoundingBox = MBoundingBox(
            MPoint(diamondBB[0][0], diamondBB[0][1], diamondBB[0][2]),
            MPoint(diamondBB[1][0], diamondBB[1][1], diamondBB[1][2]));
    }

    else if (displayIndex == 10) {  // flower
        this->theBoundingBox = MBoundingBox(
            MPoint(flowerBB[0][0], flowerBB[0][1], flowerBB[0][2]),
            MPoint(flowerBB[1][0], flowerBB[1][1], flowerBB[1][2]));
    }

    else if (displayIndex == 11) {  // jaw
        this->theBoundingBox =
            MBoundingBox(MPoint(jawBB[0][0], jawBB[0][1], jawBB[0][2]),
                         MPoint(jawBB[1][0], jawBB[1][1], jawBB[1][2]));
    }

    else if (displayIndex == 12) {  // null
        this->theBoundingBox =
            MBoundingBox(MPoint(nullBB[0][0], nullBB[0][1], nullBB[0][2]),
                         MPoint(nullBB[1][0], nullBB[1][1], nullBB[1][2]));
    }

    else if (displayIndex == 13) {  // pyramid
        this->theBoundingBox = MBoundingBox(
            MPoint(pyramidBB[0][0], pyramidBB[0][1], pyramidBB[0][2]),
            MPoint(pyramidBB[1][0], pyramidBB[1][1], pyramidBB[1][2]));
    }

    else if (displayIndex == 14) {  // sphere
        this->theBoundingBox = MBoundingBox(
            MPoint(sphereBB[0][0], sphereBB[0][1], sphereBB[0][2]),
            MPoint(sphereBB[1][0], sphereBB[1][1], sphereBB[1][2]));
    }

    else if (displayIndex == 15) {  // spine
        this->theBoundingBox =
            MBoundingBox(MPoint(spineBB[0][0], spineBB[0][1], spineBB[0][2]),
                         MPoint(spineBB[1][0], spineBB[1][1], spineBB[1][2]));
    }

    else if (displayIndex == 16) {  // square
        this->theBoundingBox = MBoundingBox(
            MPoint(squareBB[0][0], squareBB[0][1], squareBB[0][2]),
            MPoint(squareBB[1][0], squareBB[1][1], squareBB[1][2]));
    } else if (displayIndex == 17) {  // lookAt
        this->theBoundingBox = MBoundingBox(
            MPoint(lookAtBB[0][0], lookAtBB[0][1], lookAtBB[0][2]),
            MPoint(lookAtBB[1][0], lookAtBB[1][1], lookAtBB[1][2]));
    } else if (displayIndex == 18) {  // bendedArrow
        this->theBoundingBox =
            MBoundingBox(MPoint(bendedArrowBB[0][0], bendedArrowBB[0][1],
                                bendedArrowBB[0][2]),
                         MPoint(bendedArrowBB[1][0], bendedArrowBB[1][1],
                                bendedArrowBB[1][2]));
    } else if (displayIndex == 19) {  // rotateArrow
        this->theBoundingBox =
            MBoundingBox(MPoint(rotateArrowBB[0][0], rotateArrowBB[0][1],
                                rotateArrowBB[0][2]),
                         MPoint(rotateArrowBB[1][0], rotateArrowBB[1][1],
                                rotateArrowBB[1][2]));
    } else if (displayIndex == 20) {  // gear
        this->theBoundingBox =
            MBoundingBox(MPoint(gearBB[0][0], gearBB[0][1], gearBB[0][2]),
                         MPoint(gearBB[1][0], gearBB[1][1], gearBB[1][2]));
    } else if (displayIndex == 21) {  // lungs
        this->theBoundingBox =
            MBoundingBox(MPoint(lungsBB[0][0], lungsBB[0][1], lungsBB[0][2]),
                         MPoint(lungsBB[1][0], lungsBB[1][1], lungsBB[1][2]));
    }
    this->theBoundingBox.transformUsing(matPreRotate);

    if (showCenter) {
        MBoundingBox centerBB = MBoundingBox(
            MPoint(0.5 * centerScale, 0.5 * centerScale, 0.5 * centerScale),
            MPoint(-0.5 * centerScale, -0.5 * centerScale, -0.5 * centerScale));
        this->theBoundingBox.expand(centerBB);
    }
    if (showOrientation) {
        MBoundingBox showOrientationBB(
            MPoint(centerScale * lookAtBB[0][0], centerScale * lookAtBB[0][1],
                   centerScale * lookAtBB[0][2]),
            MPoint(centerScale * lookAtBB[1][0], centerScale * lookAtBB[1][1],
                   centerScale * lookAtBB[1][2]));
        this->theBoundingBox.expand(showOrientationBB);
    }
}
MStatus harbieLocator::compute(const MPlug& plug, MDataBlock& /*data*/) {
    return MS::kSuccess;
}

// called by legacy default viewport
void harbieLocator::draw(M3dView& view, const MDagPath& path,
                         M3dView::DisplayStyle style,
                         M3dView::DisplayStatus status) {
    harbieLocatorData data;
    data.checkUpdateAttr(_self);
    if (data.updateAttrs) {
        data.getPlugs(_self);
        data.getBB();
        // to draw in front
        if (data.computeCameraAdjustment) {
            MDagPath cameraPath;
            view.getCamera(cameraPath);
            data.cameraPath = cameraPath;
            data.objPath = path;
            data.getFromDagPaths();
        }
        data.getListOfPoints(_self);

        MPlug updateAttrsPlug(_self, _updateAttrs);
        updateAttrsPlug.setBool(false);
        data.updateAttrs = false;

    } else if (data.computeCameraAdjustment) {
        MDagPath cameraPath;
        view.getCamera(cameraPath);
        data.cameraPath = cameraPath;
        data.objPath = path;
        data.getFromDagPaths();
        data.updateListOfPoints();
    }

    view.beginGL();
    if ((style == M3dView::kFlatShaded) || (style == M3dView::kGouraudShaded)) {
        // Push the color settings
        //
        glPushAttrib(GL_CURRENT_BIT);

        if (status == M3dView::kActive) {
            view.setDrawColor(13, M3dView::kActiveColors);
        } else {
            view.setDrawColor(13, M3dView::kDormantColors);
        }
        /*
        MColor fColor = MHWRender::MGeometryUtilities::wireframeColor(path);
        glColor3f(GLfloat(fColor.r), GLfloat(fColor.g), GLfloat(fColor.b));
        //selected color
        */

        int nbTriangles = data.fTriangleList.size();
        MPointArray listOfTrianglesPoints;
        for (int i = 0; i < nbTriangles; ++i) {
            glBegin(GL_TRIANGLE_FAN);
            listOfTrianglesPoints = data.fTriangleList[i];
            for (int j = 0; j < listOfTrianglesPoints.length(); ++j) {
                glVertex3f(listOfTrianglesPoints[j][0],
                           listOfTrianglesPoints[j][1],
                           listOfTrianglesPoints[j][2]);
            }
            glEnd();
        }
        glPopAttrib();
    }
    // Draw the outline of the foot
    //
    MPointArray listOfLinesPoints;
    int nbLines = data.fLineList.size();
    for (int i = 0; i < nbLines; ++i) {
        glBegin(GL_LINE_STRIP);
        listOfLinesPoints = data.fLineList[i];

        int last = listOfLinesPoints.length();
        for (int j = 0; j < last; ++j) {
            glVertex3f(listOfLinesPoints[j][0], listOfLinesPoints[j][1],
                       listOfLinesPoints[j][2]);
        }
        glEnd();
    }
    view.endGL();

    // Draw the name of the harbieLocator
    /*
    view.setDrawColor( MColor( 0.1f, 0.8f, 0.8f, 1.0f ) );
    view.drawText( MString("harbieLocator"), MPoint( 0.0, 0.0, 0.0 ),
    M3dView::kCenter );
    */
}

bool harbieLocator::isBounded() const { return true; }

MBoundingBox harbieLocator::boundingBox() const {
    // Get the size
    harbieLocatorData data;
    data.checkUpdateAttr(_self);
    if (data.updateAttrs) {
        data.getPlugs(_self);
        data.getBB();
    }
    // data.get(thisMObject(), _transformMatrix);
    return data.theBoundingBox;
}

void* harbieLocator::creator() { return new harbieLocator(); }

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Viewport 2.0 override implementation
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

// By setting isAlwaysDirty to false in MPxDrawOverride constructor, the
// draw override will be updated (via prepareForDraw()) only when the node
// is marked dirty via DG evaluation or dirty propagation. Additional
// callback is also added to explicitly mark the node as being dirty (via
// MRenderer::setGeometryDrawDirty()) for certain circumstances. Note that
// the draw callback in MPxDrawOverride constructor is set to NULL in order
// to achieve better performance.
harbieLocatorDrawOverride::harbieLocatorDrawOverride(const MObject& obj)
    : MHWRender::MPxDrawOverride(obj, NULL, false) {
    fModelEditorChangedCbId = MEventMessage::addEventCallback(
        "modelEditorChanged", OnModelEditorChanged, this);

    MStatus status;
    MFnDependencyNode node(obj, &status);
    fharbieLocator =
        status ? dynamic_cast<harbieLocator*>(node.userNode()) : NULL;
}

harbieLocatorDrawOverride::~harbieLocatorDrawOverride() {
    fharbieLocator = NULL;

    if (fModelEditorChangedCbId != 0) {
        MMessage::removeCallback(fModelEditorChangedCbId);
        fModelEditorChangedCbId = 0;
    }
}

void harbieLocatorDrawOverride::OnModelEditorChanged(void* clientData) {
    // Mark the node as being dirty so that it can update on display appearance
    // switch among wireframe and shaded.
    harbieLocatorDrawOverride* ovr =
        static_cast<harbieLocatorDrawOverride*>(clientData);
    if (ovr && ovr->fharbieLocator) {
        MHWRender::MRenderer::setGeometryDrawDirty(
            ovr->fharbieLocator->thisMObject());
    }
}

MHWRender::DrawAPI harbieLocatorDrawOverride::supportedDrawAPIs() const {
    // this plugin supports both GL and DX
    return (MHWRender::kOpenGL | MHWRender::kDirectX11 |
            MHWRender::kOpenGLCoreProfile);
}

bool harbieLocatorDrawOverride::isBounded(
    const MDagPath& /*objPath*/, const MDagPath& /*cameraPath*/) const {
    return true;
}

MBoundingBox harbieLocatorDrawOverride::boundingBox(
    const MDagPath& objPath, const MDagPath& cameraPath) const {
    harbieLocatorData data;
    MObject node = objPath.node();
    data.checkUpdateAttr(node);
    if (data.updateAttrs) {
        data.getPlugs(node);
        data.getBB();
    }
    return data.theBoundingBox;
    // return MBoundingBox (MPoint(-.5, -.5, -.5), MPoint(.5, .5, .5));
}

// Called by Maya each time the object needs to be drawn.
MUserData* harbieLocatorDrawOverride::prepareForDraw(
    const MDagPath& objPath, const MDagPath& cameraPath,
    const MHWRender::MFrameContext& frameContext, MUserData* oldData) {
    // Any data needed from the Maya dependency graph must be retrieved and
    // cached in this stage. There is one cache data for each drawable instance,
    // if it is not desirable to allow Maya to handle data caching, simply
    // return null in this method and ignore user data parameter in draw
    // callback method. e.g. in this sample, we compute and cache the data for
    // usage later when we create the MUIDrawManager to draw harbieLocator in
    // method addUIDrawables().
    MStatus stat;
    harbieLocatorData* data = dynamic_cast<harbieLocatorData*>(oldData);
    MObject node = objPath.node(&stat);

    if (!data) {
        data = new harbieLocatorData;
    }
    data->checkUpdateAttr(node);

    if (data->updateAttrs) {
        data->getPlugs(node);
        data->getBB();
        if (data->computeCameraAdjustment) {
            data->cameraPath = cameraPath;
            data->objPath = objPath;
            data->getFromDagPaths();
        }
        data->getListOfPoints(node);

        MPlug updateAttrsPlug(node, harbieLocator::_updateAttrs);
        updateAttrsPlug.setBool(false);
        data->updateAttrs = false;

    } else if (data->computeCameraAdjustment) {
        data->cameraPath = cameraPath;
        data->objPath = objPath;
        data->getFromDagPaths();
        data->updateListOfPoints();
    }
    if (data->computeCameraAdjustment) {
        MHWRender::MRenderer::setGeometryDrawDirty(node);
    }

    // get correct color based on the state of object, e.g. active or dormant
    data->fColor = MHWRender::MGeometryUtilities::wireframeColor(objPath);
    return data;
}

// addUIDrawables() provides access to the MUIDrawManager, which can be used
// to queue up operations for drawing simple UI elements such as lines, circles
// and text. To enable addUIDrawables(), override hasUIDrawables() and make it
// return true.
void harbieLocatorDrawOverride::addUIDrawables(
    const MDagPath& objPath, MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& frameContext, const MUserData* data) {
    // Get data cached by prepareForDraw() for each drawable instance, then
    // MUIDrawManager can draw simple UI by these data.
    harbieLocatorData* pLocatorData = (harbieLocatorData*)data;
    if (!pLocatorData) {
        return;
    }
    /*
    if (pLocatorData->computeCameraAdjustment) {
            MHWRender::MRenderer::setGeometryDrawDirty(objPath.node());
    }
    */

    drawManager.beginDrawable();
    drawManager.setColor(pLocatorData->fColor);
    drawManager.setDepthPriority(5);

    if (frameContext.getDisplayStyle() &
        MHWRender::MFrameContext::kGouraudShaded) {
        int nbTriangles = pLocatorData->fTriangleList.size();
        for (int i = 0; i < nbTriangles; ++i) {
            drawManager.mesh(MHWRender::MUIDrawManager::kTriangles,
                             pLocatorData->fTriangleList[i]);
        }
    }
    drawManager.mesh(MHWRender::MUIDrawManager::kLines,
                     pLocatorData->fullLineList, NULL, NULL,
                     &pLocatorData->fullListIndices, NULL);
    drawManager.endDrawable();
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Plugin initialize
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

MStatus harbieLocator::initialize() {
    MFnUnitAttribute unitFn;
    MFnNumericAttribute nAttr;
    MFnEnumAttribute enumAttr;

    MStatus stat;

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
    /*
    CHECK_MSTATUS(enumAttr.addField("foot", 0));
    CHECK_MSTATUS(enumAttr.addField("box", 1));
    CHECK_MSTATUS(enumAttr.addField("rings", 2));
    */
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
    CHECK_MSTATUS(enumAttr.addField("LookAt", 17));
    CHECK_MSTATUS(enumAttr.addField("BendedArrow", 18));
    CHECK_MSTATUS(enumAttr.addField("RotateArrow", 19));
    CHECK_MSTATUS(enumAttr.addField("Gear", 20));
    CHECK_MSTATUS(enumAttr.addField("Lung", 21));

    CHECK_MSTATUS(enumAttr.setStorable(true));
    CHECK_MSTATUS(enumAttr.setKeyable(true));
    CHECK_MSTATUS(enumAttr.setReadable(true));
    CHECK_MSTATUS(enumAttr.setWritable(true));
    CHECK_MSTATUS(enumAttr.setCached(false));

    CHECK_MSTATUS(addAttribute(display));

    _showCenter =
        nAttr.create("ShowCenter", "sc", MFnNumericData::kBoolean, false);
    nAttr.setDefault(false);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setWritable(true);
    nAttr.setReadable(true);
    CHECK_MSTATUS(addAttribute(_showCenter));

    _showOrientation =
        nAttr.create("ShowOrientation", "so", MFnNumericData::kBoolean, false);
    nAttr.setDefault(false);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setWritable(true);
    nAttr.setReadable(true);
    CHECK_MSTATUS(addAttribute(_showOrientation));

    _centerScale =
        nAttr.create("CenterScale", "cc", MFnNumericData::kDouble, 1.0);
    nAttr.setDefault(1.);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setWritable(true);
    nAttr.setReadable(true);
    CHECK_MSTATUS(addAttribute(_centerScale));

    _distanceDisplay =
        nAttr.create("distanceDisplay", "dd", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(0.0);
    nAttr.setMin(0.0);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setWritable(true);
    nAttr.setReadable(true);
    CHECK_MSTATUS(addAttribute(_distanceDisplay));

    _updateAttrs =
        nAttr.create("updateAttr", "ua", MFnNumericData::kBoolean, true);
    nAttr.setStorable(true);
    CHECK_MSTATUS(addAttribute(_updateAttrs));

    return MS::kSuccess;
}
