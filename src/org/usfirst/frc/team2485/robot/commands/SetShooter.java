package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetShooter extends InstantCommand {
	
	private boolean on;
	
	public SetShooter(boolean on) {
		requires(RobotMap.shooter);
		requires(RobotMap.feeder);
		this.on = on;
	}


	@Override
	protected void initialize() {
		if (on) {
			RobotMap.shooter.setTargetSpeed(ConstantsIO.kShotRPS);
			RobotMap.feeder.setTargetSpeed(50);
		} else {
			RobotMap.shooter.disableShooter();
			RobotMap.feeder.disableFeeder();
		}
	}
	
}
