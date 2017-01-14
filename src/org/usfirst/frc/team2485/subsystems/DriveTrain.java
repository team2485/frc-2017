package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.commands.DriveWithControllers;
import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @author Nicholas Contreras
 */

public class DriveTrain extends Subsystem{
	public enum DriveSpeeds{
		SLOW_SPEED_RATING,
		NORMAL_SPEED_RATING,
		FAST_SPEED_RATING;
		
		public double getSpeedFactor(){
			return 0.6 + this.ordinal()*.02;
		}
	}
	private static final double STEERING_DEADBAND = 0.05;
	private static final double THROTTLE_DEADBAND = 0.05;
	private double driveSpeed = DriveSpeeds.NORMAL_SPEED_RATING.getSpeedFactor();
	
	public void setDriveSpeed(DriveSpeeds speed){
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

		double steering = ThresholdHandler.deadbandAndScale(controllerX, STEERING_DEADBAND, 0.01, 1);
		double throttle = ThresholdHandler.deadbandAndScale(controllerY, THROTTLE_DEADBAND, 0.01, 1);

		double leftPwm, rightPwm, overPower;
		double sensitivity = .85;

		double angularPower;
		double linearPower;

		linearPower = throttle;

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

	private void setLeftRight(double leftPwm, double rightPwm) {
		// TODO Auto-generated method stub
		leftVelocityPID.disable();
		rightVelocityPID.disable();

		RobotMap.leftDrive.set(leftPwm);
		RobotMap.rightDrive.set(rightPwm);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		Scheduler.getInstance().add(new DriveWithControllers());
	}

}
