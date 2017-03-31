package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class RunRollers extends Command {
	private static final double PWM = .5;
	private static final double CURRENT_THRESHOLD = 20;
	private static final double MIN_TIME = 1500;
	private static double startTime;
	
	@Override
	protected void initialize() {
		RobotMap.gearIntakeRoller.setManual(PWM);
		startTime = System.currentTimeMillis();
	}
	
	protected void end() {
		RobotMap.gearIntakeRoller.setManual(0);
	}

	@Override
	protected boolean isFinished() {
		return RobotMap.gearIntakeRoller.getCurrent() > CURRENT_THRESHOLD && System.currentTimeMillis() - startTime > MIN_TIME;
	}

}
