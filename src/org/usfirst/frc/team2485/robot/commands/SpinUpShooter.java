package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SpinUpShooter extends InstantCommand {
	
	private double rps;
	
	public SpinUpShooter(double rpsInput) {
		rps = rpsInput;
		requires(RobotMap.shooter);
	}

	@Override
	protected void initialize() {
		
		RobotMap.shooter.setTargetSpeed(rps);
	}
}
