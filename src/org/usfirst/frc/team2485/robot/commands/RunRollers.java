package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class RunRollers extends Command {
	private static final double PWM = .8;
	private static final double CURRENT_THRESHOLD = 10;
	private static final double MIN_TIME = 1000;
	private static final double TIME_OVER_CURRENT = 400;
	private static double startTime;
	private static long lastTimeUnderCurrent;
	
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
		if ((System.currentTimeMillis() - startTime) < MIN_TIME) {
			lastTimeUnderCurrent = System.currentTimeMillis();
			return false;
		} else {
			if (RobotMap.gearIntakeRoller.getCurrent() < CURRENT_THRESHOLD) {
				lastTimeUnderCurrent = System.currentTimeMillis();
			}
			
			return System.currentTimeMillis() - lastTimeUnderCurrent > TIME_OVER_CURRENT;
		}
	}
	
	@Override
	protected void interrupted() {
		super.interrupted();
		end();
	}

}
