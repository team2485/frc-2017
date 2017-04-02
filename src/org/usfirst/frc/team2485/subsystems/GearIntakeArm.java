package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.SetIntakeArmManual;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @author Ben Dorsey
 */

public class GearIntakeArm extends Subsystem {
	public WarlordsPIDController armPID;
	public static final double GROUND = 0, STOWED = 55, UP = 80;
	private double armAngle;
	
	public GearIntakeArm() {
		armPID = new WarlordsPIDController();
		armPID.setPID(ConstantsIO.kP_GearArm, ConstantsIO.kI_GearArm, ConstantsIO.kD_GearArm);
		armPID.setOutputs(RobotMap.gearIntakeArmMotor);
		armPID.setSources(RobotMap.gearIntakeEncoder);
		armPID.setOutputRange(-0.15, 0.5);
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
	
	public void reset() {
		setManual(0);
		armPID.disable();
	}
	
	public void setManual(double pwm) {
		armPID.disable();
		RobotMap.gearIntakeArmMotor.set(pwm);
	}
	
}
