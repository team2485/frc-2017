package org.usfirst.frc.team2485.robot.commands;


import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.InstantCommand;


public class SetRollers extends InstantCommand {
	
	boolean setRollersOn;
	
	public SetRollers(boolean setRollersOn) {
		requires(RobotMap.intakeRollers);
		this.setRollersOn = setRollersOn;
	}
	
	@Override
	protected void initialize() {
		RobotMap.intakeRollers.setManual(setRollersOn ? ConstantsIO.kIntakeRollerSpeed : 0);
	}

}
