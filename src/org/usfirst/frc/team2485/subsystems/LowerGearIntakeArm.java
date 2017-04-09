package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.SetIntakeArmManual;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class LowerGearIntakeArm extends Command {
	private static final double PWM = -.3;
	private static final long MIN_TIME = 250;
	private long startTime;
	private int timeout;
	
	public LowerGearIntakeArm(int timeout) {
		this.timeout = timeout;
		requires(RobotMap.gearIntakeArm);
	}
	
	@Override
	protected void initialize() {
		RobotMap.gearIntakeArm.setManual(PWM);
		startTime = System.currentTimeMillis();
	}
	
	protected void end() {
		RobotMap.gearIntakeArm.setManual(0);
	}
	
	@Override
	protected boolean isFinished() {
		return (System.currentTimeMillis() - startTime) > (MIN_TIME) && ((System.currentTimeMillis() - startTime) > (timeout) || Math.abs(RobotMap.gearIntakeEncoder.getRate()) < Math.PI);
	}

}
