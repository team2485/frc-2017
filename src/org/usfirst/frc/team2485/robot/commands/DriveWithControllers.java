package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.subsystems.DriveTrain.DriveSpeed;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class DriveWithControllers extends Command {
	private int mode;
	public static int hasCheeseFlag = 0x1, useCurrentFlag = 0x2, hasSteeringCorrectionFlag = 0x4, useVelocityFlag = 0x8;

	private static final boolean TRIGGER_DRIVE = true;

	public DriveWithControllers(int mode) {
		requires(RobotMap.driveTrain);
		setInterruptible(true);
		this.mode = mode;
	}

	@Override
	protected void initialize() { }

	@Override
	protected void execute() {
		if (DriverStation.getInstance().isOperatorControl()) {
			double foward = -OI.xBox.getRawAxis(1);
			double right = OI.xBox.getRawAxis(4);

			if (OI.getRightTrigger()) {
				RobotMap.driveTrain.setDriveSpeed(DriveSpeed.FAST_SPEED_RATING);
			} else if (OI.getLeftTrigger()) {
				RobotMap.driveTrain.setDriveSpeed(DriveSpeed.SLOW_SPEED_RATING);
			} else {
				RobotMap.driveTrain.setDriveSpeed(DriveSpeed.NORMAL_SPEED_RATING);
			}

			if (TRIGGER_DRIVE) {
				foward = OI.xBox.getRawAxis(OI.XBOX_RTRIGGER) - OI.xBox.getRawAxis(OI.XBOX_LTRIGGER);
				right = OI.xBox.getRawAxis(0);
			}

			RobotMap.driveTrain.warlordDrive(foward, right, (mode & useCurrentFlag) != 0, (mode & hasCheeseFlag) != 0,
					(mode & hasSteeringCorrectionFlag) != 0, (mode & useVelocityFlag) != 0);
		}

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
