package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class WheelOfDeath extends Subsystem {
	public WheelOfDeath() {
		
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	public void setSpeed(double speed) {
		RobotMap.deathMotor.set(speed);
	}
	
	public double getCurrent() {
		return RobotMap.deathMotor.getOutputCurrent();
	}

}
