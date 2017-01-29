package org.usfirst.frc.team2485.robot.commands;


import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.InstantCommand;


public class RollersOn extends InstantCommand {
	
	private double intakeRollerSpeed;

	public RollersOn() {
		requires(RobotMap.intakeRollers);
		this.intakeRollerSpeed = ConstantsIO.intakeRollerSpeed;
	}
	
	@Override
	protected void initialize() {
		RobotMap.intakeRollers.setManual(intakeRollerSpeed);
	}

}
