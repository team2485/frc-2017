package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.SetIntakeArmManual;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

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
