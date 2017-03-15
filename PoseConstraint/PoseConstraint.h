#pragma once

#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MTypeId.h>
#include <vector>


class PoseConstraint : public MPxNode {
public:
    PoseConstraint();
    ~PoseConstraint();
    static void* creator();
    static MStatus initialize();
    MPxNode::SchedulingType schedulingType() const override {return MPxNode::kParallel;}
    MStatus compute(const MPlug& plug, MDataBlock& data) override;
public:
    static MTypeId id;
    static MObject input;
    static MObject blend;
    static MObject WorldMatrix;
    static MObject parentInverseMatrix;
    static MObject localOffset;
    static MObject offset;

    static MObject output;
    static MObject translate;
    static MObject translateX;
    static MObject translateY;
    static MObject translateZ;
    static MObject rotate;
    static MObject rotateX;
    static MObject rotateY;
    static MObject rotateZ;
    static MObject scale;
    static MObject scaleX;
    static MObject scaleY;
    static MObject scaleZ;
    static MObject shear;
    static MObject shearXY;
    static MObject shearXZ;
    static MObject shearYZ;
};


