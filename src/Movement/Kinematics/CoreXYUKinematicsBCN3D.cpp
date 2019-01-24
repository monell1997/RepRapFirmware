/*
 * CoreXYUKinematicsBCN3D.cpp
 *
 *  Created on: 12 Dec 2018
 *      Author: Alejandro Garcia
 */

#include "CoreXYUKinematicsBCN3D.h"

#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/DDA.h"

CoreXYUKinematicsBCN3D::CoreXYUKinematicsBCN3D() : CoreBaseKinematics(KinematicsType::coreXYU_BCN3D)
{
}

// Return the name of the current kinematics
const char *CoreXYUKinematicsBCN3D::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "coreXYU_BCN3D" : "CoreXYU_BCN3D";
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool CoreXYUKinematicsBCN3D::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) /*override*/
{
	if (mCode == 669)
	{
		bool seen = false;
		for (size_t axis = 0; axis < CoreXYU_AXES_BCN3D; ++axis)
		{
			if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis]))
			{
				axisFactors[axis] = gb.GetFValue();
				seen = true;
			}
		}
		if (!seen && !gb.Seen('K'))
		{
			reply.printf("Kinematics is %s with axis factors", GetName(false));
			for (size_t axis = 0; axis < CoreXYU_AXES_BCN3D; ++axis)
			{
				reply.catf(" %c:%3f", reprap.GetGCodes().GetAxisLetters()[axis], (double)axisFactors[axis]);
			}
		}
		return seen;
	}

	return CoreBaseKinematics::Configure(mCode, gb, reply, error);
}

// Convert Cartesian coordinates to motor coordinates
bool CoreXYUKinematicsBCN3D::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{

	motorPos[X_AXIS] = lrintf(((machinePos[X_AXIS] * axisFactors[X_AXIS]) + (machinePos[Y_AXIS] * axisFactors[Y_AXIS])) * stepsPerMm[X_AXIS]);
	motorPos[Y_AXIS] = lrintf(((machinePos[Y_AXIS] * axisFactors[Y_AXIS])) * stepsPerMm[Y_AXIS]);
	motorPos[Z_AXIS] = lrintf(machinePos[Z_AXIS] * stepsPerMm[Z_AXIS]);
	motorPos[U_AXIS] = lrintf(((machinePos[U_AXIS] * axisFactors[U_AXIS]) - (machinePos[Y_AXIS] * axisFactors[Y_AXIS])) * stepsPerMm[U_AXIS]);


	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CoreXYUKinematicsBCN3D::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{

	machinePos[X_AXIS] = motorPos[Y_AXIS] * axisFactors[Y_AXIS] /stepsPerMm[Y_AXIS] - motorPos[X_AXIS] * axisFactors[X_AXIS]/stepsPerMm[X_AXIS];

	machinePos[Y_AXIS] = motorPos[Y_AXIS] * axisFactors[Y_AXIS] /stepsPerMm[Y_AXIS];

	machinePos[U_AXIS] = motorPos[Y_AXIS] * axisFactors[Y_AXIS]/stepsPerMm[Y_AXIS] + motorPos[U_AXIS] * axisFactors[U_AXIS]/stepsPerMm[U_AXIS];

	machinePos[Z_AXIS] = motorPos[Z_AXIS] * axisFactors[Z_AXIS]/stepsPerMm[Z_AXIS];

}

// Return true if the specified endstop axis uses shared motors.
// Used to determine whether to abort the whole move or just one motor when an endstop switch is triggered.
bool CoreXYUKinematicsBCN3D::DriveIsShared(size_t drive) const
{
	return drive == X_AXIS || drive == Y_AXIS || drive == U_AXIS
			 || drive == V_AXIS;			// V doesn't have endstop switches, but include it here just in case
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds along individual Cartesian axes have already been limited before this is called.
void CoreXYUKinematicsBCN3D::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const
{
	const float vecX = normalisedDirectionVector[0];
	const float vecY = normalisedDirectionVector[1];

	// Limit the XY motor accelerations
	const float vecMaxXY = max<float>(fabsf(vecX + vecY), fabsf(vecX - vecY));		// pick the case for the motor that is working hardest
	if (vecMaxXY > 0.01)															// avoid division by zero or near-zero
	{
		const Platform& platform = reprap.GetPlatform();
		const float aX = platform.Acceleration(0);
		const float aY = platform.Acceleration(1);
		const float vX = platform.MaxFeedrate(0);
		const float vY = platform.MaxFeedrate(1);
		const float aMax = (fabsf(vecX) + fabsf(vecY)) * aX * aY/(vecMaxXY * (fabsf(vecX) * aY + fabsf(vecY) * aX));
		const float vMax = (fabsf(vecX) + fabsf(vecY)) * vX * vY/(vecMaxXY * (fabsf(vecX) * vY + fabsf(vecY) * vX));
		dda.LimitSpeedAndAcceleration(vMax, aMax);
	}

	// Limit the UV motor accelerations
	const float vecU = normalisedDirectionVector[3];
	const float vecMaxUV = max<float>(fabsf(vecU + vecY), fabsf(vecU - vecY));		// pick the case for the motor that is working hardest
	if (vecMaxUV > 0.01)															// avoid division by zero or near-zero
	{
		const Platform& platform = reprap.GetPlatform();
		const float aU = platform.Acceleration(3);
		const float aY = platform.Acceleration(1);
		const float vU = platform.MaxFeedrate(3);
		const float vY = platform.MaxFeedrate(1);
		const float aMax = (fabsf(vecU) + fabsf(vecY)) * aU * aY/(vecMaxUV * (fabsf(vecU) * aY + fabsf(vecY) * aU));
		const float vMax = (fabsf(vecU) + fabsf(vecY)) * vU * vY/(vecMaxUV * (fabsf(vecU) * vY + fabsf(vecY) * vU));
		dda.LimitSpeedAndAcceleration(vMax, aMax);
	}
}

// End
