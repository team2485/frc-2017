package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class GearHolder extends Subsystem {
	
	public GearHolder() {
		RobotMap.gearSolenoidWings.set(false);
		RobotMap.gearSolenoidFlaps.set(false);
	}
	
	public void setWingsOpen(boolean open) {
		RobotMap.gearSolenoidWings.set(open);
	}
	
	public void setFlapsOpen(boolean open) {
		RobotMap.gearSolenoidFlaps.set(open);
	}

	@Override
	protected void initDefaultCommand() {
	}
	
	public boolean gearDetected() {
		//Not an accurate value
		return RobotMap.gearDetector.getRangeInches() < 2;
	}
}
