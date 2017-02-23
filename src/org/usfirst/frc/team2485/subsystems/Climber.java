package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

	public Climber() {
	}

	@Override
	protected void initDefaultCommand() {
	}
	
	public void climb(double power) {
		RobotMap.climberMotor.set(power);
	}

	public void stopClimbing() {
//		RobotMap.climberMotor.set(0.0);
	}

}
