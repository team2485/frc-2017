package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @author Ben Dorsey
 */

public class GearIntakeRollers extends Subsystem {

	@Override
	protected void initDefaultCommand() {
	}
	
	public void setManual(double pwm) {
		RobotMap.gearIntakeRollerMotor.set(pwm);
	}
	
	public double getCurrent() {
		return RobotMap.gearIntakeRollerMotor.getOutputCurrent();
	}
	
}
