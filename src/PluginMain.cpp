#include <maya/MFnPlugin.h>

#include "harbieCurve.h"
#include "harbieLocator.h"

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Plugin Registration
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

MStatus initializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj, PLUGIN_COMPANY, "3.0", "Any");
    // now the curve building --------------------------
    status = plugin.registerNode("makeHarbieCurve", harbieCurve::id,
                                 harbieCurve::creator, harbieCurve::initialize);
    if (!status) {
        status.perror("registerNode makeHarbieCurve");
        return status;
    }

    status = plugin.registerNode(
        "harbieLocator", harbieLocator::id, &harbieLocator::creator,
        &harbieLocator::initialize, MPxNode::kLocatorNode,
        &harbieLocator::drawDbClassification);
    if (!status) {
        status.perror("registerNode");
        return status;
    }

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        harbieLocator::drawDbClassification, harbieLocator::drawRegistrantId,
        harbieLocatorDrawOverride::Creator);
    if (!status) {
        status.perror("registerDrawOverrideCreator");
        return status;
    }

    return status;
}

MStatus uninitializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj);

    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        harbieLocator::drawDbClassification, harbieLocator::drawRegistrantId);
    if (!status) {
        status.perror("deregisterDrawOverrideCreator");
        return status;
    }

    status = plugin.deregisterNode(harbieLocator::id);
    if (!status) {
        status.perror("deregisterNode harbieLocator");
        return status;
    }

    status = plugin.deregisterNode(harbieCurve::id);
    if (!status) {
        status.perror("deregisterNode harbieCurve");
        return status;
    }

    return status;
}
