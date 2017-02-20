package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.Subsystem;

public class WheelOfDeath extends Subsystem {
	public WheelOfDeath() {
		
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	public void setCurrent(double current) {
		RobotMap.deathMotor.set(current);
	}
	
	public double getCurrent() {
		return RobotMap.deathMotor.getOutputCurrent();
	}
	
	public void updateConstants() {
		RobotMap.deathMotor.configPeakOutputVoltage(ConstantsIO.kSWODMaxVolts, -ConstantsIO.kSWODMaxVolts);
	}

}
