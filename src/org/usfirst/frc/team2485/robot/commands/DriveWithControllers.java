package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.subsystems.DriveTrain.DriveSpeed;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class DriveWithControllers extends Command {
	private boolean simple;

	public static final boolean TRIGGER_DRIVE = true;

	public DriveWithControllers(boolean simple) {
		requires(RobotMap.driveTrain);
		setInterruptible(true);
		this.simple = simple;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
		if (DriverStation.getInstance().isOperatorControl()) {
			double foward = -OI.ben.getRawAxis(OI.XBOX_AXIS_LY);
			double right = OI.ben.getRawAxis(OI.XBOX_AXIS_RX);

			if (TRIGGER_DRIVE) {
				foward = OI.ben.getRawAxis(OI.XBOX_AXIS_RTRIGGER) - OI.ben.getRawAxis(OI.XBOX_AXIS_LTRIGGER);
				right = OI.ben.getRawAxis(OI.XBOX_AXIS_LX);
			}

			RobotMap.driveTrain.warlordDrive(foward, right, simple);
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
