package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Shooter extends Subsystem {

	// Adjusted by ConstantsIO
	public WarlordsPIDController ratePID;

	public Shooter() {

		ratePID = new WarlordsPIDController();
		ratePID.setSources(RobotMap.shooterEncoder);
		ratePID.setOutputs(RobotMap.shooterMotors);
		ratePID.setBufferLength(3);
		ratePID.setOutputRange(0, 1);
		ratePID.setPID(ConstantsIO.kP_Shooter, ConstantsIO.kI_Shooter, ConstantsIO.kD_Shooter, ConstantsIO.kF_Shooter);


		disableShooter();

	}

	public boolean isPIDEnabled() {
		return (ratePID.isEnabled());
	}

	public void setTargetSpeed() {
		if (!isPIDEnabled()) {
			ratePID.enable();
		}
		ratePID.setSetpoint(ConstantsIO.kShotRPS);
		System.out.println("Shooter to: " + ConstantsIO.kShotRPS);
	}

	public void setManual(double pwm) {
		if (isPIDEnabled()) {
			ratePID.disable();
		}
		RobotMap.shooterMotors.set(pwm);
	}

	public void disableShooter() {

		if (ratePID.isEnabled()) {
			ratePID.disable();
		}
		RobotMap.shooterMotors.set(0);

	}

	public double getSetpoint() {
		return ratePID.getSetpoint();
	}

	public double getRate() {
		return RobotMap.shooterEncoder.getRate();
	}

	public double getCurrentPower() {
		return RobotMap.shooterMotors.get();
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
		ratePID.setPID(ConstantsIO.kP_Shooter, ConstantsIO.kI_Shooter, ConstantsIO.kD_Shooter, ConstantsIO.kF_Shooter);
	}

	public void reset() {
		disableShooter();
	}
}