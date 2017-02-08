package org.usfirst.frc.team2485.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.subsystems.DriveTrain.DriveSpeed;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithControllers extends Command {

	public DriveWithControllers() {
		requires(RobotMap.driveTrain);
		setInterruptible(true);
	}
	
	@Override
	protected void initialize() {

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

		RobotMap.driveTrain.warlordDrive(foward, right, true);

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
