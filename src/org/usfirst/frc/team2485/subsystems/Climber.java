package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

	public Climber() {
		
	}
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	public void climb() {
		RobotMap.climberMotor.set(ConstantsIO.kClimberSpeed);
	}
	
	public void stopClimbing() {
		RobotMap.climberMotor.set(0.0);
	}

}
