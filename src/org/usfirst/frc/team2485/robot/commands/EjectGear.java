package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class EjectGear extends Command {
	private static final double PWM = -.5;
	private static final double MIN_TIME = 1000;
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
		return System.currentTimeMillis() - startTime > MIN_TIME;
	}

}
