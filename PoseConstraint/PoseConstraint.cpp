

#include "PoseConstraint.h"
#include <maya/MGlobal.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MEulerRotation.h>
#include <maya/MQuaternion.h>

#include <algorithm>


MTypeId PoseConstraint::id(0x00121BD2);
MObject PoseConstraint::input;
MObject PoseConstraint::blend;
MObject PoseConstraint::WorldMatrix;
MObject PoseConstraint::parentInverseMatrix;
MObject PoseConstraint::localOffset;
MObject PoseConstraint::offset;

MObject PoseConstraint::output;
MObject PoseConstraint::translate;
MObject PoseConstraint::translateX;
MObject PoseConstraint::translateY;
MObject PoseConstraint::translateZ;
MObject PoseConstraint::rotate;
MObject PoseConstraint::rotateX;
MObject PoseConstraint::rotateY;
MObject PoseConstraint::rotateZ;
MObject PoseConstraint::scale;
MObject PoseConstraint::scaleX;
MObject PoseConstraint::scaleY;
MObject PoseConstraint::scaleZ;
MObject PoseConstraint::shear;
MObject PoseConstraint::shearX;
MObject PoseConstraint::shearY;
MObject PoseConstraint::shearZ;


PoseConstraint::PoseConstraint() {}

PoseConstraint::~PoseConstraint() {}

void* PoseConstraint::creator() {
    return new PoseConstraint();
}

MStatus PoseConstraint::initialize() {
    MStatus stat;
    MFnNumericAttribute fnNum;
    MFnUnitAttribute fnUnit;
    MFnMatrixAttribute fnMat;
    MFnCompoundAttribute fnComp;

    // Input Attributes
    offset = fnMat.create("offset", "off", MFnMatrixAttribute::kDouble);
    fnMat.setKeyable(true);
    addAttribute(offset);

    WorldMatrix = fnMat.create("WorldMatrix", "wmat", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS(stat);
    fnMat.setKeyable(true);

    blend = fnNum.create("blend", "blnd", MFnNumericData::kDouble, 1.0, &stat);
    CHECK_MSTATUS(stat);
    fnNum.setMin(0.0);
    fnNum.setMax(1.0);
    fnNum.setKeyable(true);

    localOffset = fnMat.create("localOffset", "loff", MFnMatrixAttribute::kDouble);
    fnMat.setKeyable(true);

    input = fnComp.create("Input", "in", &stat);
    CHECK_MSTATUS(stat);
    fnComp.addChild(WorldMatrix);
    fnComp.addChild(blend);
    fnComp.addChild(localOffset);
    fnComp.setArray(true);
    fnComp.setUsesArrayDataBuilder(true);
    addAttribute(input);

    parentInverseMatrix = fnMat.create("parentInverseMatrix", "pinv", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS(stat);
    fnMat.setKeyable(true);
    addAttribute(parentInverseMatrix);

    // Output Attributes
    translateX = fnNum.create("translateX", "trax", MFnNumericData::kDouble, 0.0);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    translateY = fnNum.create("translateY", "tray", MFnNumericData::kDouble, 0.0);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    translateZ = fnNum.create("translateZ", "traz", MFnNumericData::kDouble, 0.0);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    translate = fnNum.create("translate", "tra", translateX, translateY, translateZ);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    rotateX = fnUnit.create("rotateX", "rotx", MFnUnitAttribute::kAngle, 0.0);
    fnUnit.setWritable(false);
    fnUnit.setStorable(false);

    rotateY = fnUnit.create("rotateY", "roty", MFnUnitAttribute::kAngle, 0.0);
    fnUnit.setWritable(false);
    fnUnit.setStorable(false);

    rotateZ = fnUnit.create("rotateZ", "rotz", MFnUnitAttribute::kAngle, 0.0);
    fnUnit.setWritable(false);
    fnUnit.setStorable(false);

    rotate = fnNum.create("rotate", "rot", rotateX, rotateY, rotateZ);
    fnNum.setWritable(false);

    scaleX = fnNum.create("scaleX", "sclx", MFnNumericData::kDouble, 1.0);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    scaleY = fnNum.create("scaleY", "scly", MFnNumericData::kDouble, 1.0);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    scaleZ = fnNum.create("scaleZ", "sclz", MFnNumericData::kDouble, 1.0);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    scale = fnNum.create("scale", "scl", scaleX, scaleY, scaleZ);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    shearX = fnNum.create("shearX", "shrx", MFnNumericData::kDouble, 1.0);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    shearY = fnNum.create("shearY", "shry", MFnNumericData::kDouble, 1.0);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    shearZ = fnNum.create("shearZ", "shrz", MFnNumericData::kDouble, 1.0);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    shear = fnNum.create("shear", "shr", shearX, shearY, shearZ);
    fnNum.setWritable(false);
    fnNum.setStorable(false);

    output = fnComp.create("output", "out");
    fnComp.setWritable(false);
    fnComp.addChild(translate);
    fnComp.addChild(rotate);
    fnComp.addChild(scale);
    fnComp.addChild(shear);
    CHECK_MSTATUS(stat);
    addAttribute(output);

    attributeAffects(WorldMatrix, translate);
    attributeAffects(WorldMatrix, translateX);
    attributeAffects(WorldMatrix, translateY);
    attributeAffects(WorldMatrix, translateZ);
    attributeAffects(WorldMatrix, rotate);
    attributeAffects(WorldMatrix, rotateX);
    attributeAffects(WorldMatrix, rotateY);
    attributeAffects(WorldMatrix, rotateZ);
    attributeAffects(WorldMatrix, scale);
    attributeAffects(WorldMatrix, scaleX);
    attributeAffects(WorldMatrix, scaleY);
    attributeAffects(WorldMatrix, scaleZ);
    attributeAffects(WorldMatrix, shear);
    attributeAffects(WorldMatrix, shearX);
    attributeAffects(WorldMatrix, shearY);
    attributeAffects(WorldMatrix, shearZ);

    attributeAffects(localOffset, translate);
    attributeAffects(localOffset, translateX);
    attributeAffects(localOffset, translateY);
    attributeAffects(localOffset, translateZ);
    attributeAffects(localOffset, rotate);
    attributeAffects(localOffset, rotateX);
    attributeAffects(localOffset, rotateY);
    attributeAffects(localOffset, rotateZ);
    attributeAffects(localOffset, scale);
    attributeAffects(localOffset, scaleX);
    attributeAffects(localOffset, scaleY);
    attributeAffects(localOffset, scaleZ);
    attributeAffects(localOffset, shear);
    attributeAffects(localOffset, scaleX);
    attributeAffects(localOffset, scaleY);
    attributeAffects(localOffset, shearZ);

    attributeAffects(offset, translate);
    attributeAffects(offset, translateX);
    attributeAffects(offset, translateY);
    attributeAffects(offset, translateZ);
    attributeAffects(offset, rotate);
    attributeAffects(offset, rotateX);
    attributeAffects(offset, rotateY);
    attributeAffects(offset, rotateZ);
    attributeAffects(offset, scale);
    attributeAffects(offset, scaleX);
    attributeAffects(offset, scaleY);
    attributeAffects(offset, scaleZ);
    attributeAffects(offset, shear);
    attributeAffects(offset, scaleX);
    attributeAffects(offset, scaleY);
    attributeAffects(offset, shearZ);

    attributeAffects(blend, translate);
    attributeAffects(blend, translateX);
    attributeAffects(blend, translateY);
    attributeAffects(blend, translateZ);
    attributeAffects(blend, rotate);
    attributeAffects(blend, rotateX);
    attributeAffects(blend, rotateY);
    attributeAffects(blend, rotateZ);
    attributeAffects(blend, scale);
    attributeAffects(blend, scaleX);
    attributeAffects(blend, scaleY);
    attributeAffects(blend, scaleZ);
    attributeAffects(blend, shear);
    attributeAffects(blend, shearX);
    attributeAffects(blend, shearY);
    attributeAffects(blend, shearZ);

    attributeAffects(parentInverseMatrix, translate);
    attributeAffects(parentInverseMatrix, translateX);
    attributeAffects(parentInverseMatrix, translateY);
    attributeAffects(parentInverseMatrix, translateZ);
    attributeAffects(parentInverseMatrix, rotate);
    attributeAffects(parentInverseMatrix, rotateX);
    attributeAffects(parentInverseMatrix, rotateY);
    attributeAffects(parentInverseMatrix, rotateZ);
    attributeAffects(parentInverseMatrix, scale);
    attributeAffects(parentInverseMatrix, scaleX);
    attributeAffects(parentInverseMatrix, scaleY);
    attributeAffects(parentInverseMatrix, scaleZ);
    attributeAffects(parentInverseMatrix, shear);
    attributeAffects(parentInverseMatrix, shearX);
    attributeAffects(parentInverseMatrix, shearY);
    attributeAffects(parentInverseMatrix, shearZ);

    return MS::kSuccess;
}

MStatus PoseConstraint::compute(const MPlug& plug, MDataBlock& data) {
    if (plug!=translate && plug!=rotate && plug!=scale && plug!=shear)
        return MS::kUnknownParameter;

    // inputs
    MArrayDataHandle inputArrayHandle = data.inputArrayValue(input);
    MMatrix invParentMat = data.inputValue(parentInverseMatrix).asMatrix();
    MMatrix offsetTfm = data.inputValue(offset).asMatrix();

    // process
    MTransformationMatrix tfm = invParentMat.inverse();
    MDataHandle component;
    double wtA, wtB;

    // a
    MVector posA = tfm.getTranslation(MSpace::kTransform);
    MQuaternion rotA = tfm.rotation();
    double sclA[3];
    tfm.getScale(sclA, MSpace::kTransform);
    double shrA[3];
    tfm.getShear(shrA, MSpace::kTransform);

    // b
    MVector posB;
    MQuaternion rotB;
    double sclB[3];
    double shrB[3];

    // interpolate ab
    for (unsigned int i = 0; i < inputArrayHandle.elementCount(); ++i) {
        component = inputArrayHandle.inputValue();
        wtB = component.child(blend).asDouble();
        tfm = component.child(localOffset).asMatrix() * component.child(WorldMatrix).asMatrix();

        posB = tfm.getTranslation(MSpace::kTransform);
        rotB = tfm.rotation();
        tfm.getScale(sclB, MSpace::kTransform);
        tfm.getShear(shrB, MSpace::kTransform);

        wtA = 1.0 - wtB;
        posA = (posA * wtA) + (posB * wtB);
        rotA = slerp(rotA, rotB, wtB);
        sclA[0] = (sclA[0] * wtA) + (sclB[0] * wtB);
        sclA[1] = (sclA[1] * wtA) + (sclB[1] * wtB);
        sclA[2] = (sclA[2] * wtA) + (sclB[2] * wtB);

        shrA[0] = (shrA[0] * wtA) + (shrB[0] * wtB);
        shrA[1] = (shrA[1] * wtA) + (shrB[1] * wtB);
        shrA[2] = (shrA[2] * wtA) + (shrB[2] * wtB);

        inputArrayHandle.next();
    }

    // build the tfm
    MTransformationMatrix outTfm;
    outTfm.setScale(sclA, MSpace::kTransform);
    outTfm.setRotationQuaternion(rotA[0], rotA[1], rotA[2], rotA[3], MSpace::kTransform);
    outTfm.setTranslation(posA, MSpace::kTransform);
    outTfm.setShear(shrA, MSpace::kTransform);
    outTfm = offsetTfm * outTfm.asMatrix() * invParentMat;

    // ouputs
    posA = outTfm.translation(MSpace::kTransform);
    MEulerRotation rot = outTfm.eulerRotation();
    outTfm.getScale(sclA, MSpace::kTransform);
    outTfm.getShear(shrA, MSpace::kTransform);

    MDataHandle outTra = data.outputValue(translate);
    MDataHandle outRot = data.outputValue(rotate);
    MDataHandle outScl = data.outputValue(scale);
    MDataHandle outShr = data.outputValue(shear);

    // set
    outTra.setMVector(posA);
    outRot.set(rot.x, rot.y, rot.z);
    outScl.set(sclA[0], sclA[1], sclA[2]);
    outShr.set(shrA[0], shrA[1], shrA[2]);

    // set all the things clean
    data.setClean(translate);
    data.setClean(translateX);
    data.setClean(translateY);
    data.setClean(translateZ);
    data.setClean(rotate);
    data.setClean(rotateX);
    data.setClean(rotateY);
    data.setClean(rotateZ);
    data.setClean(scale);
    data.setClean(scaleX);
    data.setClean(scaleY);
    data.setClean(scaleZ);
    data.setClean(shear);
    data.setClean(shearX);
    data.setClean(shearY);
    data.setClean(shearZ);
    return MS::kSuccess;
}

