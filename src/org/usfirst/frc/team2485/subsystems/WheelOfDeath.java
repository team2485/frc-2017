package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public class WheelOfDeath extends Subsystem {

	private static final double MAX_EFFICIENCY = .75;
	private static final int MAX_EFFICIENCY_SPEED = 292;
	private static final double STALL_TORQUE = 0.71;
	private static final int FREE_SPEED = 312;

	private double previousSpeed;
	
	public WheelOfDeath() {
		stop();
	}

	@Override
	protected void initDefaultCommand() {
	}

	
	public void setCurrent(double current) {
		RobotMap.deathMotor.changeControlMode(TalonControlMode.Current);
		RobotMap.deathMotor.set(current);
		// System.out.println("Set current: " + current);
	}

	public double getCurrent() {
		return RobotMap.deathMotor.getOutputCurrent();
	}

	public void updateConstants() {
		
	}

	public void setPWM(double pwm) {
		RobotMap.deathMotor.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.deathMotor.set(pwm);
	}

	public void stop() {
		RobotMap.deathMotor.changeControlMode(TalonControlMode.PercentVbus);
		RobotMap.deathMotor.set(0);
	}

	public double getSpeed() {
		double curEfficiency = previousSpeed / MAX_EFFICIENCY_SPEED * MAX_EFFICIENCY;
		curEfficiency = Math.max(curEfficiency, 0.01);
//		System.out.println("Current Efficiency: " + curEfficiency);
		
		double electricalPower = RobotMap.deathMotor.getOutputCurrent() * RobotMap.deathMotor.getOutputVoltage();
//		System.out.println("Current: " + RobotMap.deathMotor.getOutputCurrent());
//		System.out.println("Voltage: " + RobotMap.deathMotor.getOutputVoltage());
//		System.out.println("Electrical Power: " + electricalPower);
		double mechanicalPower = electricalPower * curEfficiency;
//		System.out.println("Mechanical Power: " + mechanicalPower);
		double torque = (1 - (previousSpeed / FREE_SPEED)) * STALL_TORQUE;
//		System.out.println("Torque: " + torque);

		double curSpeed = mechanicalPower / torque * 2 * Math.PI;
//		System.out.println("Current speed: " + curSpeed);

		return curSpeed;
	}
	
	public double getPWM() {
		return RobotMap.deathMotor.get();
	}
	
	public void setLastSpeed(double lastSpeed) {
		previousSpeed = lastSpeed;
	}
}
