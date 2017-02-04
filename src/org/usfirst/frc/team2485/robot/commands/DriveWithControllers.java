package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.subsystems.DriveTrain.DriveSpeed;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithControllers extends Command {
	private boolean isCurrent;

	public DriveWithControllers(boolean isCurrent) {
		requires(RobotMap.driveTrain);
		setInterruptible(true);
		this.isCurrent = isCurrent;
	}

	@Override
	protected void initialize() {
		if (isCurrent) {
			RobotMap.driveLeft1.changeControlMode(TalonControlMode.Current);
			RobotMap.driveLeft2.changeControlMode(TalonControlMode.Current);
			RobotMap.driveLeft3.changeControlMode(TalonControlMode.Current);
			RobotMap.driveRight1.changeControlMode(TalonControlMode.Current);
			RobotMap.driveRight2.changeControlMode(TalonControlMode.Current);
			RobotMap.driveRight3.changeControlMode(TalonControlMode.Current);
		}else{
			RobotMap.driveLeft1.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveLeft2.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveLeft3.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveRight1.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveRight2.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveRight3.changeControlMode(TalonControlMode.PercentVbus);	
		}
	}

	@Override
	protected void execute() {

		double foward = -OI.xBox.getRawAxis(1);
		double right = OI.xBox.getRawAxis(4);

		if (OI.getRightTrigger()) {
			RobotMap.driveTrain.setDriveSpeed(DriveSpeed.FAST_SPEED_RATING);
		} else if (OI.getLeftTrigger()) {
			RobotMap.driveTrain.setDriveSpeed(DriveSpeed.SLOW_SPEED_RATING);
		} else {
			RobotMap.driveTrain.setDriveSpeed(DriveSpeed.NORMAL_SPEED_RATING);
		}

		RobotMap.driveTrain.warlordDrive(foward, right, isCurrent);

	}

	@Override
	protected void interrupted() {
		end();
	}

	@Override
	protected void end() {
		RobotMap.driveTrain.setDriveSpeed(DriveSpeed.NORMAL_SPEED_RATING);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
