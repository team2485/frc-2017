package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class RunRollers extends Command {
	private static final double PWM = .5;
	private static final double CURRENT_THRESHOLD = 5;
	
	@Override
	protected void initialize() {
		RobotMap.gearIntakeRollers.setManual(PWM);
	}
	
	protected void end() {
		RobotMap.gearIntakeRollers.setManual(0);
	}

	@Override
	protected boolean isFinished() {
		return RobotMap.gearIntakeRollers.getCurrent() > CURRENT_THRESHOLD;
	}

}
