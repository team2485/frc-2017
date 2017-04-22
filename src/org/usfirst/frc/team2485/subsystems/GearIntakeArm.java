package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.SetIntakeArmManual;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.PIDOutputWrapper;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @author Ben Dorsey
 */

public class GearIntakeArm extends Subsystem {
	public WarlordsPIDController armPID = new WarlordsPIDController();
	public WarlordsPIDController armRatePid = new WarlordsPIDController();
	private PIDOutputWrapper armPidOutput = new PIDOutputWrapper();

	public static final double GROUND = 0, STOWED = 49, UP = 80;
	public GearIntakeArm() {
		armPID.setPID(ConstantsIO.kP_GearArm, ConstantsIO.kI_GearArm, ConstantsIO.kD_GearArm);
		armPID.setOutputs(armPidOutput);
		armPID.setSources(RobotMap.gearIntakeEncoder);
		armPID.setOutputRange(-0.35, 0.35);
		
		armRatePid.setPID(ConstantsIO.kP_GearArmRate, ConstantsIO.kI_GearArmRate, ConstantsIO.kD_GearArmRate,
				ConstantsIO.kF_GearArmRate);
		armRatePid.setOutputs(armPidOutput);
		armRatePid.setSources(RobotMap.gearIntakeEncoderRate);
		armRatePid.setOutputRange(-0.4, 0.4);
		
		armPidOutput.setPidOutput((double out) -> {
			double angle = RobotMap.gearIntakeEncoder.getDistance() * 360.0 / 250;
			RobotMap.gearIntakeArmMotor.set(out + 0.35 * Math.cos(Math.toRadians(angle)));
		});
	}
	
	
	@Override
	protected void initDefaultCommand() {
		this.setDefaultCommand(new SetIntakeArmManual());
	}
	
	public void zeroEncoder() {
		RobotMap.gearIntakeEncoder.reset();
	}
	
	
	public void setSetpoint(double setpoint) {
		armPID.enable();
		armPID.setSetpoint(setpoint);
		armPID.setPercentTolerance(.05);;
	}
	
	public void setRate(double rate) {
		armRatePid.enable();
		armRatePid.setSetpoint(rate);
	}
	
	public void reset() {
		setManual(0);
		armPID.disable();
	}
	
	public void setManual(double pwm) {
		armPID.disable();
		RobotMap.gearIntakeArmMotor.set(pwm);
	}
	
	public double getError() {
		return armPID.getAvgError();
	}
	
}
