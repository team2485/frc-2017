
package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends IterativeRobot {

	public void robotInit() {
		ConstantsIO.init();
		RobotMap.init();
		OI.init();
		RobotMap.updateConstants();
	}

	public void disabledInit() {
		// Reset and disable subsytems
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		
		updateSmartDashboard();
	}

	public void autonomousInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();

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
		
	}
}
