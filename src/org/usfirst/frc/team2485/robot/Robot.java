
package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public void robotInit() {
		ConstantsIO.init();
		RobotMap.init();
		OI.init();
		RobotMap.updateConstants();
	}

	public void disabledInit() {
		
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		
		updateSmartDashboard();
	}

	public void autonomousInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
		RobotMap.driveTrain.setRate(.5, .5);
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();

	}

	public void teleopInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
	}

	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		updateSmartDashboard();
		System.out.println("Left Drive Encoder Rate: " + RobotMap.driveEncRight.getRate());
		System.out.println("Right Drive Encoder Rate: " + RobotMap.driveEncRight.getRate());
		
		
	}

	public void testInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
	}

	public void testPeriodic() {

//		if (RobotMap.pressureSwitch.get()) {
//			RobotMap.compressorSpike.set(Relay.Value.kOff);
//		} else {
//			RobotMap.compressorSpike.set(Relay.Value.kForward);
//		}
		
		updateSmartDashboard();

	}

	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Right Talon Current", RobotMap.driveTalonRight.getOutputCurrent());
		SmartDashboard.putNumber("Left Talon Current", RobotMap.driveTalonLeft.getOutputCurrent());
	}
}
