
package org.usfirst.frc.team2485.robot;

import java.util.Timer;
import java.util.TimerTask;

import org.opencv.core.Core;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	private boolean cameraFound;
                  
	
	public void robotInit() {
		
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		ConstantsIO.init();
		RobotMap.init();
		OI.init();
		
		RobotMap.updateConstants();
		
	}

	public void disabledInit() {
		RobotMap.driveTrain.reset();
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
//		RobotMap.driveTrain.setLeftRightVelocity(-20, -20);

		ConstantsIO.init();
		RobotMap.updateConstants();
	}
	

	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		updateSmartDashboard();
//		int curRent = (int)(edu.wpi.first.wpilibj.Timer.getFPGATimestamp()%2)+1;
//		RobotMap.driveTrainLeft.set(curRent);
		RobotMap.driveTrainLeft.set(8);
		RobotMap.driveTrainRight.set(8);
		//RobotMap.driveTrainRight.set(curRent);	
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
//		SmartDashboard.putNumber("avg error", RobotMap.driveTrain.getLeftVelocityAvgError());
		SmartDashboard.putNumber("Left Current", RobotMap.driveLeft3.getOutputCurrent());
		SmartDashboard.putNumber("Right Current", RobotMap.driveRight3.getOutputCurrent());
	}
}
