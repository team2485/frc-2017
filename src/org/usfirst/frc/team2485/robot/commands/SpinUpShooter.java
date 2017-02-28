package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SpinUpShooter extends InstantCommand {
	
	
	public SpinUpShooter() {
		requires(RobotMap.shooter);
	}

	@Override
	protected void initialize() {
		System.out.println("Shooter");
		RobotMap.shooter.setTargetSpeed();
	}
}
