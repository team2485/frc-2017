package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class SetIntakeArm extends Command {
	private double setpoint;
	
	public SetIntakeArm(double setpoint) {
		this.setpoint = setpoint;
	}
	
	@Override
	protected void initialize() {
		RobotMap.gearIntakeArm.setSetpoint(setpoint);
		SetIntakeArmManual.passiveControl = false;
		
	}

	@Override
	protected boolean isFinished() {
		return RobotMap.gearIntakeArm.armPID.isOnTarget();
	}
}
