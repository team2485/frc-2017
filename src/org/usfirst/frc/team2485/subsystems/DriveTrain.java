package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

/**
* @author Nicholas Contreras
*/

public class DriveTrain {

	private static final double STEERING_DEADBAND = 0.1;
	private static final double THROTTLE_DEADBAND = 0.1;
	
	private WarlordsPIDController leftVelocityPID, rightVelocityPID;
	
	public enum DriveSpeed {
		SLOW_SPEED_RATING, NORMAL_SPEED_RATING, FAST_SPEED_RATING;
		
		public double getSpeedFactor() {
			return 0.6 + this.ordinal() * 0.2;
		}
	}
	
	private double driveSpeed = DriveSpeed.NORMAL_SPEED_RATING.getSpeedFactor();

	public void setDriveSpeed(DriveSpeed speed) {
		driveSpeed = speed.getSpeedFactor();
	}
	
	/**
	 * W.A.R. Lord Drive This drive method is based off of Team 254's Ultimate
	 * Ascent cheesyDrive code.
	 *
	 * @param controllerY
	 *            controllerY should be positive for forward motion
	 * @param controllerX
	 */
	public void warlordDrive(double controllerY, double controllerX) {

		boolean isQuickTurn = OI.xBox.getRawButton(OI.XBOX_RBUMPER);

		boolean isHighGear = isQuickTurn;

		double steeringNonLinearity;

		double steering = ThresholdHandler.deadbandAndScale(controllerX,
				STEERING_DEADBAND, 0.01, 1);
		double throttle = ThresholdHandler.deadbandAndScale(controllerY,
				THROTTLE_DEADBAND, 0.01, 1);

		double angularPower;
		double linearPower;
		
		linearPower = throttle;
		
		double leftPwm, rightPwm, overPower;
		double sensitivity = 0.85;

		// Quickturn!
		if (isQuickTurn) {
			overPower = 1.0;
			angularPower = steering;
		} else {
			overPower = 0.0;
			angularPower = throttle * steering * sensitivity;
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
		
		leftPwm *= driveSpeed;
		rightPwm *= driveSpeed;

		setLeftRight(leftPwm, rightPwm);
	}
	
	/**
	 * Sends outputs values to the left and right side of the drive base after
	 * scaling based on virtual gear. <br>
	 * The parameters should both be positive to move forward. One side has
	 * inverted motors...do not send a negative to one side and a positive to
	 * the other for forward or backwards motion.
	 *
	 * @param leftOutput
	 * @param rightOutput
	 */
	public void setLeftRight(double leftOutput, double rightOutput) {

		leftVelocityPID.disable();
		rightVelocityPID.disable();

		RobotMap.leftDrive.set(leftOutput);
		RobotMap.rightDrive.set(rightOutput);
	}
}
