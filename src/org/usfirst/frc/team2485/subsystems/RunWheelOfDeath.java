package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
/**
 * 
 * @author Vicky
 *
 */

public class RunWheelOfDeath extends Command {
	private static final double MAX_CURRENT = 5;
	private double speed;
	private boolean done;

	public RunWheelOfDeath(double speed) {
		this.speed = speed;
	}
	
	@Override
	protected void initialize() {
		RobotMap.wheelOfDeath.setSpeed(speed);
		done = false;
	}
	
	@Override
	protected void execute() {
		double current = RobotMap.wheelOfDeath.getCurrent();
		if(current > MAX_CURRENT) {
			done = true;
		}
	}
	@Override
	protected boolean isFinished() {
		return done;
	}
	
	@Override
	protected void end() {
		RobotMap.wheelOfDeath.setSpeed(0);
	}
}
