package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.util.ThresholdHandler;

/**
* @author Nicholas Contreras
*/

public class DriveTrain {

	/**
	 * W.A.R. Lord Drive This drive method is based off of Team 254's Ultimate
	 * Ascent cheesyDrive code.
	 *
	 * @param controllerY
	 *            controllerY should be positive for forward motion
	 * @param controllerX
	 */
	public void warlordDrive(double controllerY, double controllerX) {

		if (OI.xBox.getRawAxis(OI.XBOX_LTRIGGER) > 0.4) {
			driveSpeed = SLOW_SPEED_RATING;
		} else if (OI.xBox.getRawAxis(OI.XBOX_RTRIGGER) > 0.4) {
			driveSpeed = FAST_SPEED_RATING;
		} else {
			driveSpeed = NORMAL_SPEED_RATING;
		}

		isQuickTurn = OI.xBox.getRawButton(OI.XBOX_RBUMPER);

		boolean isHighGear = isQuickTurn;

		double steeringNonLinearity;

		double steering = ThresholdHandler.deadbandAndScale(controllerX,
				STEERING_DEADBAND, 0.01, 1);
		double throttle = ThresholdHandler.deadbandAndScale(controllerY,
				THROTTLE_DEADBAND, 0.01, 1);

		double negInertia = steering - oldSteering;
		oldSteering = steering;

		if (isHighGear) {
			steeringNonLinearity = 0.6;
			// Apply a sin function that's scaled to make it feel better.
			steering = Math
					.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
					/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);
			steering = Math
					.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
					/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);
		} else {
			steeringNonLinearity = 0.5;
			// Apply a sin function that's scaled to make it feel better.
			steering = Math
					.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
					/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);
			steering = Math
					.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
					/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);
			steering = Math
					.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
					/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);
		}

		double leftPwm, rightPwm, overPower;
		double sensitivity = 1.7;

		double angularPower;
		double linearPower;

		// Negative inertia!
		double negInertiaAccumulator = 0.0;
		double negInertiaScalar;
		if (isHighGear) {
			negInertiaScalar = 5.0;
			sensitivity = SENSITIVITY_HIGH;
		} else {
			if (steering * negInertia > 0) {
				negInertiaScalar = 2.5;
			} else {
				if (Math.abs(steering) > 0.65) {
					negInertiaScalar = 5.0;
				} else {
					negInertiaScalar = 3.0;
				}
			}
			sensitivity = SENSITIVITY_LOW;
		}
		double negInertiaPower = negInertia * negInertiaScalar;
		negInertiaAccumulator += negInertiaPower;

		steering = steering + negInertiaAccumulator;
		linearPower = throttle;

		// Quickturn!
		if (isQuickTurn) {
			if (Math.abs(linearPower) < 0.2) {
				double alpha = 0.1;
				steering = steering > 1 ? 1.0 : steering;
				quickStopAccumulator = (1 - alpha) * quickStopAccumulator
						+ alpha * steering * 0.5;
			}
			overPower = 1.0;
			if (isHighGear) {
				sensitivity = 1.0;
			} else {
				sensitivity = 1.0;
			}
			angularPower = steering;
		} else {
			overPower = 0.0;
			angularPower = throttle * steering * sensitivity
					- quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0.0;
			}
		}

		rightPwm = leftPwm = linearPower;

		leftPwm += angularPower;
		rightPwm -= angularPower;

		if (leftPwm > 1.0) {
			rightPwm -= overPower * (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
			leftPwm -= overPower * (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
			rightPwm += overPower * (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
			leftPwm += overPower * (-1.0 - rightPwm);
			rightPwm = -1.0;
		}

		setLeftRight(leftPwm, rightPwm);
	}
	
}
