/*
 * CoreBaseKinematicsBCN3D.h
 *
 *  Created on: 12 Dec 2018
 *      Author: Alejandro Garcia
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREXYUKINEMATICSBCN3D_H_
#define SRC_MOVEMENT_KINEMATICS_COREXYUKINEMATICSBCN3D_H_

#include "CoreBaseKinematics.h"

class CoreXYUKinematicsBCN3D : public CoreBaseKinematics
{
public:
	CoreXYUKinematicsBCN3D();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool DriveIsShared(size_t drive) const override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const override;
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREXYUKINEMATICSBCN3D_H_ */
