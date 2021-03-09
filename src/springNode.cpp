/*

MGEAR is under the terms of the MIT License

Copyright (c) 2016 Jeremie Passerin, Miquel Campos

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author:     Jeremie Passerin      geerem@hotmail.com  www.jeremiepasserin.com
Author:     Miquel Campos         hello@miquel-campos.com  www.miquel-campos.com
Date:       2016 / 10 / 10

*/

/////////////////////////////////////////////////
// INCLUDE
/////////////////////////////////////////////////
#include "mgear_solvers.h"
#include <maya/MFnCompoundAttribute.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MArrayDataBuilder.h>

/////////////////////////////////////////////////
// GLOBAL
/////////////////////////////////////////////////
MTypeId		mgear_springNode::id(0x0011FEC1);

//Static variables

MObject mgear_springNode::aOutput;
MObject mgear_springNode::aGoal;
MObject mgear_springNode::aGoalX;
MObject mgear_springNode::aGoalY;
MObject mgear_springNode::aGoalZ;

MObject mgear_springNode::aDamping;
MObject mgear_springNode::aStiffness;
MObject mgear_springNode::aTime;

MObject mgear_springNode::aGravity;

MObject mgear_springNode::aCollider;
MObject mgear_springNode::aColliderX;
MObject mgear_springNode::aColliderY;
MObject mgear_springNode::aColliderZ;
MObject mgear_springNode::aColliderRadius;
MObject mgear_springNode::aColliderList;
MObject mgear_springNode::aColliderSoftness;
MObject mgear_springNode::aUseGroundPlane;
MObject mgear_springNode::aGroundPlaneHeight;

//MObject mgear_springNode::aParentInverse;
MObject mgear_springNode::aSpringIntensity;
MObject mgear_springNode::aSpringActive;


mgear_springNode::mgear_springNode(){}
mgear_springNode::~mgear_springNode(){}

mgear_springNode::SchedulingType mgear_springNode::schedulingType() const
{
	return kParallel;
}

void* mgear_springNode::creator()
{
	return new mgear_springNode();
}



//INIT
MStatus mgear_springNode::initialize()
{
	MStatus status;
	MFnNumericAttribute nAttr;
	MFnUnitAttribute uAttr;
	MFnMatrixAttribute mAttr;
	MFnCompoundAttribute cmpAttr;


	aOutput = nAttr.createPoint("output", "out"); // initializing attribute
	nAttr.setWritable(false);
	nAttr.setStorable(false);
	nAttr.setReadable(true);
	addAttribute(aOutput); // adding the attribute to the node

	aGoal = nAttr.createPoint("goal", "goal");
	aGoalX = nAttr.child(0);
	aGoalY = nAttr.child(1);
	aGoalZ = nAttr.child(2);
	nAttr.setKeyable(true);
	nAttr.setStorable(false);
	addAttribute(aGoal);
	attributeAffects(aGoal, aOutput);

	aTime = uAttr.create("time", "time", MFnUnitAttribute::kTime, 0.0f);
	addAttribute(aTime);
	attributeAffects(aTime, aOutput);

	aStiffness = nAttr.create("stiffness", "stiffness", MFnNumericData::kFloat, 1.0f);
	nAttr.setKeyable(true);
	nAttr.setMin(0.0f);
	nAttr.setMax(1.0f);
	addAttribute(aStiffness);
	attributeAffects(aStiffness, aOutput);

	aDamping = nAttr.create("damping", "damping", MFnNumericData::kFloat, 1.0f);
	nAttr.setKeyable(true);
	nAttr.setMin(0.0f);
	nAttr.setMax(1.0f);
	addAttribute(aDamping);
	attributeAffects(aDamping, aOutput);

	aSpringIntensity = nAttr.create("intensity", "intensity", MFnNumericData::kFloat, 1.0f);
	nAttr.setKeyable(true);
	nAttr.setMin(0.0f);
	nAttr.setMax(1.0f);
	addAttribute(aSpringIntensity);
	attributeAffects(aSpringIntensity, aOutput);

	aSpringActive = nAttr.create("active", "active", MFnNumericData::kFloat, 1.0f);
	nAttr.setKeyable(true);
	nAttr.setMin(0.0f);
	nAttr.setMax(1.0f);
	addAttribute(aSpringActive);
	attributeAffects(aSpringActive, aOutput);

	aGravity = nAttr.create("gravity", "gravity", MFnNumericData::kFloat, 0.0f);
	nAttr.setKeyable(true);
	nAttr.setMin(-1000.0f);
	nAttr.setMax(1000.0f);
	addAttribute(aGravity);
	attributeAffects(aGravity, aOutput);

	aColliderSoftness = nAttr.create("collide_softness", "collide_softness", MFnNumericData::kFloat, 0.5f);
	nAttr.setKeyable(true);
	nAttr.setMin(0.0f);
	nAttr.setMax(1.0f);
	addAttribute(aColliderSoftness);
	attributeAffects(aColliderSoftness, aOutput);

	aUseGroundPlane = nAttr.create("use_ground", "use_ground", MFnNumericData::kBoolean, 0);
	nAttr.setKeyable(true);
	addAttribute(aUseGroundPlane);
	attributeAffects(aUseGroundPlane, aOutput);

	aGroundPlaneHeight = nAttr.create("ground_height", "ground_height", MFnNumericData::kFloat, 0.0f);
	nAttr.setKeyable(true);
	addAttribute(aGroundPlaneHeight);
	attributeAffects(aGroundPlaneHeight, aOutput);


	aCollider = nAttr.createPoint("collider", "collider");
	aColliderX = nAttr.child(0);
	aColliderY = nAttr.child(1);
	aColliderZ = nAttr.child(2);
	nAttr.setKeyable(true);
 	McheckErr(nAttr.setDisconnectBehavior(MFnAttribute::kDelete), "setDisconnectBehavior");
	addAttribute(aCollider);
	attributeAffects(aCollider, aOutput);


	aColliderRadius = nAttr.create("collide_radius", "collide_radius", MFnNumericData::kFloat, 0.0f);
	nAttr.setKeyable(true);
	nAttr.setMin(-1000.0f);
	nAttr.setMax(1000.0f);
 	McheckErr(nAttr.setDisconnectBehavior(MFnAttribute::kDelete), "setDisconnectBehavior");
	addAttribute(aColliderRadius);
	attributeAffects(aColliderRadius, aOutput);


	aColliderList = cmpAttr.create("colliders_list", "colliders_list", &status);
	cmpAttr.setArray(true);
	cmpAttr.setKeyable(true);
	McheckErr(cmpAttr.addChild(aCollider), "compoundAttr.addChild");
	McheckErr(cmpAttr.addChild(aColliderRadius), "compoundAttr.addChild");
 	McheckErr(cmpAttr.setDisconnectBehavior(MFnAttribute::kDelete), "setDisconnectBehavior");
 	McheckErr(addAttribute(aColliderList), "addAttribute");
	McheckErr(attributeAffects(aColliderList, aOutput), "attributeAffects");



	/*aParentInverse = mAttr.create("parentInverse", "parentInverse");
	mAttr.setStorable(true);
	mAttr.setKeyable(true);
	mAttr.setConnectable(true);
	addAttribute(aParentInverse);
	attributeAffects(aParentInverse, aOutput);*/

	// bool _initialized = false;

	return MS::kSuccess;
}

// COMPUTE

MStatus mgear_springNode::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus status;

	if (plug != aOutput)
	{
		return MS::kUnknownParameter;
	}



	// getting inputs attributes
	float damping = data.inputValue(aDamping, &status).asFloat();
	float stiffness = data.inputValue(aStiffness, &status).asFloat();
	float gravity = data.inputValue(aGravity, &status).asFloat();

	//MVector goal = data.inputValue(aGoal, &status).asVector();
	MDataHandle h;
	MVector goal;
	h = data.inputValue(aGoal, &status);
	McheckStatusAndReturnIt(status);
	goal = h.asFloatVector();


	MTime currentTime = data.inputValue(aTime, &status).asTime();
	//MMatrix parentInverse = data.inputValue(aParentInverse, &status).asMatrix();
	float springIntensity = data.inputValue(aSpringIntensity, &status).asFloat();
	float springActive = data.inputValue(aSpringActive, &status).asFloat();


	if (_initialized == false) {
		// Initialize the point states
		//MGlobal::displayInfo( "mc_spring: Initialize the point states" );
		_previousPosition = goal;
		_currentPosition = goal;
		_previousTime = currentTime;
		_initialized = true;
		//return MS::kSuccess;
	}
	// Check if the timestep is just 1 frame since we want a stable simulation
	double timeDifference = currentTime.value() - _previousTime.value();
	if (timeDifference > 1.0 || timeDifference < 0.0) {
		_initialized = false;
		_previousPosition = goal;
		_currentPosition = goal;
		_previousTime = currentTime;
		//MGlobal::displayInfo( "mc_spring: time checker, reset position" );
		//return MS::kSuccess;
	}

	// computation
	MFloatVector velocity = (_currentPosition - _previousPosition) * (1.0 - damping);
	MVector newPosition = _currentPosition + velocity;
	MFloatVector goalForce = (goal - newPosition) * stiffness;
	goalForce.y += gravity;
	newPosition += goalForce;



	// Apply collision
	float colliderStrength = 1.0 - data.inputValue(aColliderSoftness, &status).asFloat();
	MVector colliderPos;
	MVector distFromCollider;
	float colliderRadius;
  float collideAmount;
	MArrayDataHandle arrayHandle = data.inputArrayValue(aColliderList, &status);
	McheckErr(status, "arrayHandle construction for aColliderList failed\n");
	unsigned count = arrayHandle.elementCount();
	for( int i = 0; i < count; i++) {
		arrayHandle.jumpToArrayElement(i);
		h = arrayHandle.inputValue(&status);
		McheckErr(status, "handle evaluation failed\n");
		colliderPos = h.child(aCollider).asFloatVector();
		colliderRadius = h.child(aColliderRadius).asFloat();
		distFromCollider = newPosition - colliderPos;
		collideAmount = (colliderRadius - distFromCollider.length()) / distFromCollider.length() * colliderStrength;
		collideAmount = collideAmount > 0 ? collideAmount : 0;   // Only push outward
		newPosition += collideAmount * distFromCollider; // Move radially
	}


	bool useGroundPlane = data.inputValue(aUseGroundPlane, &status).asBool();
	float groundPlaneHeight = data.inputValue(aGroundPlaneHeight, &status).asFloat();

	float distFromGround = (newPosition[1] - groundPlaneHeight);
	float collideAmount = -1.0f * distFromGround * colliderStrength;
	collideAmount = collideAmount > 0 ? collideAmount : 0;
	newPosition[1] += ((float)useGroundPlane) * collideAmount;


	// store the states for the next calculation
	_previousPosition = _currentPosition;
	_currentPosition = newPosition;
	_previousTime = currentTime;

	//multipply the position by the spring intensity
	//calculamos depues de los states, para no afectarlos
	newPosition = goal + (((newPosition - goal) * springIntensity) * springActive);

	//Setting the output in local space
	// esto lo hacemos depues de hacer el store de los states

	//newPosition *=  parentInverse;


	MDataHandle hOutput = data.outputValue(aOutput, &status);
	McheckStatusAndReturnIt(status);
	hOutput.set3Float(newPosition.x, newPosition.y, newPosition.z);
	hOutput.setClean();
	data.setClean(plug);

	return MS::kSuccess;

}
