package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * @author Ben Dorsey
 */

public class SetIntakeArm extends InstantCommand {
	private double setpoint;
	
	public SetIntakeArm(double setpoint) {
		this.setpoint = setpoint;
	}
	
	@Override
	protected void initialize() {
		RobotMap.gearIntakeArm.setSetpoint(setpoint);
	}
}
