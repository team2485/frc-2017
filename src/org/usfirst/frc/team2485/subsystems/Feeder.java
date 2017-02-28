package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Feeder extends Subsystem {

	public WarlordsPIDController ratePID;

	public Feeder() {

		ratePID = new WarlordsPIDController();
		ratePID.setSources(RobotMap.feederEncoder);
		ratePID.setOutputs(RobotMap.feederMotor);
		ratePID.setBufferLength(3);
		ratePID.setOutputRange(0, 1);

		ratePID.setPID(ConstantsIO.kP_Feeder, ConstantsIO.kI_Feeder, ConstantsIO.kD_Feeder, ConstantsIO.kF_Feeder);

		disableFeeder();

	}

	public boolean isPIDEnabled() {
		return (ratePID.isEnabled());
	}

	public void setTargetSpeed(double ips) {
		if (!isPIDEnabled()) {
			ratePID.enable();
		}
		ratePID.setSetpoint(ips);
	}

	public void setManual(double pwm) {
		if (isPIDEnabled()) {
			ratePID.disable();
		}
		RobotMap.feederMotor.set(pwm);
	}

	public void disableFeeder() {

		if (ratePID.isEnabled()) {
			ratePID.disable();
		}

		RobotMap.feederMotor.set(0);

	}

	public double getSetpoint() {
		return ratePID.getSetpoint();
	}

	public double getRate() {
		return RobotMap.feederEncoder.getRate();
	}

	public double getCurrentPower() {
		return RobotMap.feederMotor.get();
	}

	public double getError() {

		return getSetpoint() - getRate();

	}

	public double getAvgError() {

		return ratePID.getAvgError();

	}

	@Override
	protected void initDefaultCommand() {
	}

	public void updateConstants() {
		ratePID.setPID(ConstantsIO.kP_Feeder, ConstantsIO.kI_Feeder, ConstantsIO.kD_Feeder, ConstantsIO.kF_Feeder);
	}

	public void reset() {
		disableFeeder();
	}
}