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
Author:     Jascha Wohlkinger     jwohlkinger@gmail.com
Date:       2020/ 11 / 06

*/

#include "mgear_solvers.h"

MTypeId mgear_matrixConstraint::id(0x75759);

// ---------------------------------------------------
// input plugs
// ---------------------------------------------------
MObject mgear_matrixConstraint::aDriverMatrix;
MObject mgear_matrixConstraint::aDrivenParentInverseMatrix;
MObject mgear_matrixConstraint::aDrivenRestMatrix;

// ---------------------------------------------------
// output plugs
// ---------------------------------------------------
MObject mgear_matrixConstraint::aOutputMatrix;

MObject mgear_matrixConstraint::aTranslate;
MObject mgear_matrixConstraint::aTranslateX;
MObject mgear_matrixConstraint::aTranslateY;
MObject mgear_matrixConstraint::aTranslateZ;

MObject mgear_matrixConstraint::aRotate;
MObject mgear_matrixConstraint::aRotateX;
MObject mgear_matrixConstraint::aRotateY;
MObject mgear_matrixConstraint::aRotateZ;

MObject mgear_matrixConstraint::aScale;
MObject mgear_matrixConstraint::aScaleX;
MObject mgear_matrixConstraint::aScaleY;
MObject mgear_matrixConstraint::aScaleZ;

MObject mgear_matrixConstraint::aShear;
MObject mgear_matrixConstraint::aShearX;
MObject mgear_matrixConstraint::aShearY;
MObject mgear_matrixConstraint::aShearZ;

mgear_matrixConstraint::mgear_matrixConstraint()
{
}

mgear_matrixConstraint::~mgear_matrixConstraint()
{
}

void* mgear_matrixConstraint::creator()
{
	return (new mgear_matrixConstraint());
}

MStatus mgear_matrixConstraint::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus status;

	// -----------------------------------------
	// input attributes
	// -----------------------------------------
	MMatrix driver_matrix = data.inputValue(aDriverMatrix, &status).asMatrix();
	MMatrix driven_inverse_matrix = data.inputValue(aDrivenParentInverseMatrix, &status).asMatrix();
	MMatrix rest_matrix = data.inputValue(aDrivenRestMatrix, &status).asMatrix();

	// -- our needed variables
	MTransformationMatrix result;
	double scale[3];
	double shear[3];

	MMatrix mult_matrix = driver_matrix * driven_inverse_matrix;
	
	// -- multiply the result of the mult matrix by the rest
	// -- need to have the rotation calculated seperaltely - (joint orientation)
	MMatrix rotate_matrix = mult_matrix * rest_matrix.inverse();

	MTransformationMatrix matrix(mult_matrix);
	MTransformationMatrix rotate_tfm(rotate_matrix);
	
	// -- decompose the matrix values to construct into the final matrix
	MVector translation = matrix.getTranslation(MSpace::kWorld);
	matrix.getScale(scale, MSpace::kWorld);
	matrix.getShear(shear, MSpace::kWorld);
	
	// -- the quaternion rotation of the rotate matrix
	MQuaternion rotation = rotate_tfm.rotation();

	// -- compose our matrix
	result.setTranslation(translation, MSpace::kWorld);
	result.setRotationQuaternion(rotation.x, rotation.y, rotation.z, rotation.w);
	result.setScale(scale, MSpace::kWorld);
	result.setShear(shear, MSpace::kWorld);

	// -----------------------------------------
	// output
	// -----------------------------------------

	// -- our final output variables
	double scale_result[3] = { 1.0, 1.0, 1.0 };
	double shear_result[3];

	MDataHandle matrix_handle = data.outputValue(aOutputMatrix, &status);
	matrix_handle.setMMatrix(result.asMatrix());
	data.setClean(aOutputMatrix);

	MDataHandle translate_handle = data.outputValue(aTranslate, &status);
	translate_handle.setMVector(result.getTranslation(MSpace::kWorld));

	MEulerRotation rotation_result = result.eulerRotation();
	MDataHandle rotate_handle = data.outputValue(aRotate, &status);
	rotate_handle.set3Double(rotation_result.x, rotation_result.y, rotation_result.z);

	result.getScale(scale_result, MSpace::kWorld);
	MDataHandle scale_handle = data.outputValue(aScale, &status);
	scale_handle.set3Double(scale_result[0], scale_result[1], scale_result[2]);

	result.getShear(shear_result, MSpace::kWorld);
	MDataHandle shear_handle = data.outputValue(aShear, &status);
	shear_handle.set3Double(shear_result[0], shear_result[1], shear_result[2]);

	data.setClean(plug);

	return MS::kSuccess;
}

MStatus mgear_matrixConstraint::initialize()
{
	MStatus status;

	MFnMatrixAttribute mAttr;
	MFnNumericAttribute nAttr;
	MFnUnitAttribute uAttr;

	// -----------------------------------------
	// input attributes
	// -----------------------------------------
	aDriverMatrix = mAttr.create("driverMatrix", "driverMatrix", MFnMatrixAttribute::kDouble);
	mAttr.setKeyable(true);
	mAttr.setReadable(false);
	mAttr.setWritable(true);
	mAttr.setStorable(true);

	aDrivenParentInverseMatrix = mAttr.create("drivenParentInverseMatrix", "drivenParentInverseMatrix", MFnMatrixAttribute::kDouble);
	mAttr.setKeyable(true);
	mAttr.setReadable(false);
	mAttr.setWritable(true);
	mAttr.setStorable(true);

	aDrivenRestMatrix = mAttr.create("drivenRestMatrix", "drivenRestMatrix", MFnMatrixAttribute::kDouble);
	mAttr.setKeyable(true);
	mAttr.setReadable(false);
	mAttr.setWritable(true);
	mAttr.setStorable(true);

	// -----------------------------------------
	// output attributes
	// -----------------------------------------
	aOutputMatrix = mAttr.create("outputMatrix", "outputMatrix", MFnMatrixAttribute::kDouble);
	mAttr.setKeyable(false);
	mAttr.setWritable(true);
	mAttr.setStorable(false);

	// -- out translation
	aTranslateX = nAttr.create("translateX", "translateX", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	nAttr.setStorable(true);

	aTranslateY = nAttr.create("translateY", "translateY", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	nAttr.setStorable(true);

	aTranslateZ = nAttr.create("translateZ", "translateZ", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	nAttr.setStorable(true);

	aTranslate = nAttr.create("translate", "translate", aTranslateX, aTranslateY, aTranslateZ);
	nAttr.setWritable(false);

	// -- out rotation
	aRotateX = uAttr.create("rotateX", "rotateX", MFnUnitAttribute::kAngle);
	uAttr.setWritable(false);
	uAttr.setStorable(true);

	aRotateY = uAttr.create("rotateY", "rotateY", MFnUnitAttribute::kAngle);
	uAttr.setWritable(false);
	uAttr.setStorable(true);

	aRotateZ = uAttr.create("rotateZ", "rotateZ", MFnUnitAttribute::kAngle);
	uAttr.setWritable(false);
	uAttr.setStorable(true);

	aRotate = nAttr.create("rotate", "rotate", aRotateX, aRotateY, aRotateZ);
	nAttr.setWritable(false);

	// -- out scale
	aScaleX = nAttr.create("scaleX", "scaleX", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	nAttr.setStorable(true);

	aScaleY = nAttr.create("scaleY", "scaleY", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	nAttr.setStorable(true);

	aScaleZ = nAttr.create("scaleZ", "scaleZ", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	nAttr.setStorable(true);

	aScale = nAttr.create("scale", "scale", aScaleX, aScaleY, aScaleZ);
	nAttr.setWritable(false);

	// -- out shear
	aShearX = nAttr.create("shearX", "shearX", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	nAttr.setStorable(true);

	aShearY = nAttr.create("shearY", "shearY", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	nAttr.setStorable(true);

	aShearZ = nAttr.create("shearZ", "shearZ", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	nAttr.setStorable(true);

	aShear = nAttr.create("shear", "shear", aShearX, aShearY, aShearZ);
	nAttr.setWritable(false);

	// -----------------------------------------
	// add attributes
	// -----------------------------------------
	addAttribute(aDriverMatrix);
	addAttribute(aDrivenParentInverseMatrix);
	addAttribute(aDrivenRestMatrix);

	addAttribute(aOutputMatrix);
	
	addAttribute(aTranslate);
	addAttribute(aTranslateX);
	addAttribute(aTranslateY);
	addAttribute(aTranslateZ);

	addAttribute(aRotate);
	addAttribute(aRotateX);
	addAttribute(aRotateY);
	addAttribute(aRotateZ);

	addAttribute(aScale);
	addAttribute(aScaleX);
	addAttribute(aScaleY);
	addAttribute(aScaleZ);

	addAttribute(aShear);
	addAttribute(aShearX);
	addAttribute(aShearY);
	addAttribute(aShearZ);

	// -----------------------------------------
	// attribute affects
	// -----------------------------------------
	attributeAffects(aDriverMatrix, aOutputMatrix);
	attributeAffects(aDrivenParentInverseMatrix, aOutputMatrix);
	attributeAffects(aDrivenRestMatrix, aOutputMatrix);

	attributeAffects(aDriverMatrix, aTranslate);
	attributeAffects(aDriverMatrix, aRotate);
	attributeAffects(aDriverMatrix, aScale);
	attributeAffects(aDriverMatrix, aShear);

	attributeAffects(aDrivenParentInverseMatrix, aTranslate);
	attributeAffects(aDrivenParentInverseMatrix, aRotate);
	attributeAffects(aDrivenParentInverseMatrix, aScale);
	attributeAffects(aDrivenParentInverseMatrix, aShear);

	attributeAffects(aDrivenRestMatrix, aTranslate);
	attributeAffects(aDrivenRestMatrix, aRotate);
	attributeAffects(aDrivenRestMatrix, aScale);
	attributeAffects(aDrivenRestMatrix, aShear);

	return MS::kSuccess;
}

mgear_matrixConstraint::SchedulingType mgear_matrixConstraint::schedulingType() const
{
	return kParallel;
}
