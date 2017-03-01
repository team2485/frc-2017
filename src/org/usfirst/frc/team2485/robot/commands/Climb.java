package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class Climb extends InstantCommand {
	
	private boolean on; 
	
	public Climb(boolean on) {
		requires(RobotMap.climber);
		this.on = on;
	}
	
	@Override
	public void initialize() {
		RobotMap.climber.setPower(on ? ConstantsIO.kClimberPower : 0);
	}
	
}
