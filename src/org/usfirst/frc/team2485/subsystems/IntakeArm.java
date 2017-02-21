package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class IntakeArm extends Subsystem {

	@Override
	protected void initDefaultCommand() {
	}
	
	public void setHorizontal(boolean isExtended) {
		RobotMap.intakeArmSolenoidHorizontal.set(isExtended);
	}
	
	
	public void setVertical(boolean isExtended) {
		RobotMap.intakeArmSolenoidVertical1.set(isExtended);
		RobotMap.intakeArmSolenoidVertical2.set(isExtended);
	}

}
