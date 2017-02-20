package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class IntakeArm extends Subsystem {

	@Override
	protected void initDefaultCommand() {
	}
	
	public void extend() {
		RobotMap.intakeArmSolenoid.set(true);
	}
	
	public void reset() {
		RobotMap.intakeArmSolenoid.set(false);
	}
	
	public boolean isExtended() {
		return RobotMap.intakeArmSolenoid.get();
	}

}
