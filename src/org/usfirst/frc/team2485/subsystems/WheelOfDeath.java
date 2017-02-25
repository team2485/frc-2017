package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public class WheelOfDeath extends Subsystem {
	public WheelOfDeath() {

	}

	@Override
	protected void initDefaultCommand() {
	}

	public void setCurrent(double current) {
		RobotMap.deathMotor.changeControlMode(TalonControlMode.Current);
		RobotMap.deathMotor.set(current);
//		System.out.println("Set current: " + current);
	}

	public double getCurrent() {
		return RobotMap.deathMotor.getOutputCurrent();
	}

	public void updateConstants() {
		RobotMap.deathMotor.setPID(ConstantsIO.kP_SWODCurrent, ConstantsIO.kI_SWODCurrent, 
				ConstantsIO.kD_SWODCurrent, ConstantsIO.kF_SWODCurrent, 0, 0, 0);
		RobotMap.deathMotor.configPeakOutputVoltage(ConstantsIO.kSWODMaxVolts, -ConstantsIO.kSWODMaxVolts);
	}
	
	public void setPWM(double pwm) {
		RobotMap.deathMotor.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.deathMotor.set(pwm);

	}
	
	public void stop() {
		RobotMap.deathMotor.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.deathMotor.set(0);
	}

}
