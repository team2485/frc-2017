
package org.usfirst.frc.team2485.robot;

import org.opencv.core.Mat;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
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
		RobotMap.driveTrain.reset();
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		
		updateSmartDashboard();
	}

	public void autonomousInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
		UsbCamera usb = CameraServer.getInstance().startAutomaticCapture(0);
		usb.setExposureManual(0);
		usb.setBrightness(0);
	}
	
	Mat newMat = new Mat();
	
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
		
		SmartDashboard.putBoolean("Test Boolean", System.currentTimeMillis() % 1000 > 500);
	}
}
