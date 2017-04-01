package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class SetIntakeArmManual extends Command {
	
	public SetIntakeArmManual() {
		requires(RobotMap.gearIntakeArm);
		this.setInterruptible(true);
	}
	
	@Override
	protected void execute() {
		if (Math.abs(OI.elliot.getRawAxis(OI.XBOX_AXIS_LY)) > .2) {
			RobotMap.gearIntakeArm.setManual(OI.elliot.getRawAxis(OI.XBOX_AXIS_LY)/2);
		} else {
			if (!RobotMap.gearIntakeArm.armPID.isEnabled()) {
				RobotMap.gearIntakeArm.setSetpoint(RobotMap.gearIntakeEncoder.get());
			}
		}
	}
	
	
	
	@Override
	protected boolean isFinished() {
		return false;
	}

}
