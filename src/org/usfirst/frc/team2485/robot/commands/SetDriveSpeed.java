package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.subsystems.DriveTrain.DriveSpeed;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * @author Nicholas Contreras
 */

public class SetDriveSpeed extends InstantCommand {

	private DriveSpeed speed;

	public SetDriveSpeed(DriveSpeed speed) {
		this.speed = speed;
	}

	@Override
	protected void initialize() {
		RobotMap.driveTrain.setDriveSpeed(speed);
	}
}
