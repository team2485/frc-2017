package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @author Ben Dorsey
 */

public class GearIntakeArm extends Subsystem {
	private WarlordsPIDController armPID;
	public static final double GROUND = 0, STOWED = 55;
	
	public GearIntakeArm() {
		armPID = new WarlordsPIDController();
		armPID.setPID(ConstantsIO.kP_GearArm, ConstantsIO.kI_GearArm, ConstantsIO.kD_GearArm);
		armPID.setOutputs(RobotMap.gearIntakeArmMotor);
		armPID.setSources(RobotMap.gearIntakeEncoder);
		armPID.setOutputRange(-0.15, 0.5);
	}
	
	
	@Override
	protected void initDefaultCommand() {
	}
	
	public void zeroEncoder() {
		RobotMap.gearIntakeEncoder.reset();
	}
	
	public void setSetpoint(double setpoint) {
		armPID.enable();
		armPID.setSetpoint(setpoint);
	}
	
	public void reset() {
		setManual(0);
		armPID.disable();
	}
	
	public void setManual(double pwm) {
		armPID.disable();
		RobotMap.gearIntakeArmMotor.set(pwm);
	}
	
}
