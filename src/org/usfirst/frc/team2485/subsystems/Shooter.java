package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;


public class Shooter extends Subsystem {
	

	//Adjusted by ConstantsIO
	public static double RPS_SHOT = 95; 
	public WarlordsPIDController ratePID;

	public Shooter() {

		

		ratePID = new WarlordsPIDController(RobotMap.shooterEncoder, RobotMap.shooterMotor);
		ratePID.setBufferLength(3);
		ratePID.setOutputRange(0, 1);
		ratePID.setPID(ConstantsIO.kP_Shooter, ConstantsIO.kI_Shooter, ConstantsIO.kD_Shooter,
				ConstantsIO.kF_Shooter);
		
		RPS_SHOT = ConstantsIO.kShotRPS;		

		disableShooter();

	}
	
	public boolean isPIDEnabled() {
		return (ratePID.isEnabled());
	}
	
	public void setTargetSpeed(double rpm) {
		if (!isPIDEnabled()){
			ratePID.enable();
		}
		ratePID.setSetpoint(rpm);
	}
	
	public void setManual(double pwm) {
		if (isPIDEnabled()) {
			ratePID.disable();
		}
		RobotMap.shooterMotor.set(pwm);
	}
	
	public void disableShooter() {

		if (ratePID.isEnabled()) {
			ratePID.disable();
		}
		RobotMap.shooterMotor.set(0);

	}

	public double getSetpoint() {
		return ratePID.getSetpoint();
	}

	public double getRate() {
		return RobotMap.shooterEncoder.getRate();
	}
	
	public double getCurrentPower() {
		return RobotMap.shooterMotor.get();
	}

	public double getError() {

		return getSetpoint() - getRate();

	}

	public double getAvgError() {

		return ratePID.getAvgError();

	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

	public void updateConstants() {
		ratePID.setPID(ConstantsIO.kP_Shooter, ConstantsIO.kI_Shooter, ConstantsIO.kD_Shooter,
				ConstantsIO.kF_Shooter);
		
				RPS_SHOT = ConstantsIO.kShotRPS;
	}
	
	public void reset() {
		disableShooter();
	}
}