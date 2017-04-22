package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class SetIntakeArmManual extends Command {
	public static boolean passiveControl;
	private long lastTimeUnderCurrent;
	private static double MAX_CURRENT = 10;
	private static long OVER_CURRENT_TIME = 500;
	public SetIntakeArmManual() {
		requires(RobotMap.gearIntakeArm);
		this.setInterruptible(true);
	}
	
	@Override
	protected void execute() {
		double val = ThresholdHandler.deadbandAndScale(OI.elliot.getRawAxis(OI.XBOX_AXIS_RY), 0.15, 0.15, 0.3) + 
				ThresholdHandler.deadbandAndScale(OI.elliot.getRawAxis(OI.XBOX_AXIS_LY), 0.2, 0.01, 0.2);
		if (val != 0) {
			double angle = RobotMap.gearIntakeEncoder.getDistance() * 360.0 / 250;
			RobotMap.gearIntakeArm.setManual(val + 0.35 * Math.cos(Math.toRadians(angle)));
			passiveControl = false;
		} else {
			
			if (RobotMap.gearIntakeArmMotor.getOutputCurrent() < MAX_CURRENT) {
				lastTimeUnderCurrent = System.currentTimeMillis();
			}
			
			if (System.currentTimeMillis() - lastTimeUnderCurrent > OVER_CURRENT_TIME || (RobotMap.gearIntakeArm.armPID.getSetpoint() < 10 && RobotMap.gearIntakeArm.armPID.isEnabled())) {
				passiveControl = true;
			}
			
			if (passiveControl) {
				RobotMap.gearIntakeArm.setManual(0);
			} else if (!RobotMap.gearIntakeArm.armPID.isEnabled()) {
				RobotMap.gearIntakeArm.setSetpoint(RobotMap.gearIntakeEncoder.get());
			}
		}
	}
	
	
	
	@Override
	protected boolean isFinished() {
		return false;
	}

}
