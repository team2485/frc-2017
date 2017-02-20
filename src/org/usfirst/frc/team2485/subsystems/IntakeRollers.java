package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class IntakeRollers extends Subsystem {
	
	public IntakeRollers() {
		disableIntakeRollers();
	}

	public void setManual(double pwm) {
		RobotMap.intakeMotor.set(pwm);
	}

	public void disableIntakeRollers() {
		RobotMap.intakeMotor.set(0);
	}

	public double getCurrentPower() {
		return RobotMap.intakeMotor.get();
	}


	@Override
	protected void initDefaultCommand() {
		
	}

	public void updateConstants() {

	}

	public void reset() {
		disableIntakeRollers();
	}
}