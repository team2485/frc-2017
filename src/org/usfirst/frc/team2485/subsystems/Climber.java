package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

	@Override
	protected void initDefaultCommand() { }
	
	public void setPower(double power) {
		RobotMap.climberMotor.set(power);
	}

}
