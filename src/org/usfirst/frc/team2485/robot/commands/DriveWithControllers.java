package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.subsystems.DriveTrain.DriveSpeed;
import org.usfirst.frc.team2485.util.ThresholdHandler;

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
	protected void execute() {
		if (DriverStation.getInstance().isOperatorControl()) {
			double foward = -OI.ben.getRawAxis(OI.XBOX_AXIS_LY);
			double steering = ThresholdHandler.deadbandAndScale(OI.ben.getRawAxis(OI.XBOX_AXIS_RX), DriveTrain.STEERING_DEADBAND, 0, 1);
;

			if (TRIGGER_DRIVE) {
				foward = OI.ben.getRawAxis(OI.XBOX_AXIS_RTRIGGER) - OI.ben.getRawAxis(OI.XBOX_AXIS_LTRIGGER);
				steering += ThresholdHandler.deadbandAndScale(OI.ben.getRawAxis(OI.XBOX_AXIS_LX), DriveTrain.STEERING_DEADBAND, 0.0, 0.75);
			}

			RobotMap.driveTrain.warlordDrive(foward, steering, simple);
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
