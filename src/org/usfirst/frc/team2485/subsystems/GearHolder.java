package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class GearHolder extends Subsystem {
	
	public GearHolder() {
		
	}
	public void setBottomOpen(boolean open) {
		
		RobotMap.gearSolenoidBottom1.set(open);
		RobotMap.gearSolenoidBottom2.set(!open);
		
	}
	
	public void setChuteOpen(boolean open) {
		
		RobotMap.gearSolenoidTop1.set(open);
		RobotMap.gearSolenoidTop2.set(!open);
		
	}

	@Override
	protected void initDefaultCommand() {

	}
	
	public boolean gearDetected() {
		//Not an accurate value
		return RobotMap.gearDetector.getRangeInches() < 2;
	}
	
}
