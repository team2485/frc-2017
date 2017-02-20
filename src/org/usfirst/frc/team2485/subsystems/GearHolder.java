package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class GearHolder extends Subsystem {
	
	public GearHolder() {
	}
	
	public void setBottomOpen(boolean open) {
		RobotMap.gearSolenoidBottom.set(open);
	}
	
	public void setChuteOpen(boolean open) {
		RobotMap.gearSolenoidTop.set(open);
	}

	@Override
	protected void initDefaultCommand() {
	}
	
	public boolean gearDetected() {
		//Not an accurate value
		return RobotMap.gearDetector.getRangeInches() < 2;
	}
}
