package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.subsystems.DriveTrain.DriveSpeed;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class DriveWithControllers extends Command {
	private int mode;
	public static int hasCheeseFlag = 0x1, useCurrentFlag = 0x2, hasSteeringCorrectionFlag = 0x4, useVelocityFlag = 0x8;

	public static final boolean TRIGGER_DRIVE = true;

	public DriveWithControllers(int mode) {
		requires(RobotMap.driveTrain);
		setInterruptible(true);
		this.mode = mode;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
		if (DriverStation.getInstance().isOperatorControl()) {
			double foward = -OI.xBox.getRawAxis(OI.XBOX_AXIS_LY);
			double right = OI.xBox.getRawAxis(OI.XBOX_AXIS_RX);

			if (TRIGGER_DRIVE) {
				foward = OI.xBox.getRawAxis(OI.XBOX_AXIS_RTRIGGER) - OI.xBox.getRawAxis(OI.XBOX_AXIS_LTRIGGER);
				right = OI.xBox.getRawAxis(OI.XBOX_AXIS_LX);
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
