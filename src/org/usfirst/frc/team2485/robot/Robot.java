
package org.usfirst.frc.team2485.robot;

import org.opencv.core.Mat;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	private AutoPath path;

	public void robotInit() {

		ConstantsIO.init();
		RobotMap.init();
		OI.init();
		path = new AutoPath(AutoPath.getPointsForFunction((double t) -> {
			t *= Math.PI;
			return new Pair((Math.cos(t) - 1) * 27, (Math.sin(t) * 27));
		}, 10000));

		RobotMap.updateConstants();

	}

	public void disabledInit() {
		Scheduler.getInstance().removeAll();
		RobotMap.driveTrain.reset();
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
	}

	public void autonomousInit() {
		ConstantsIO.init();
		RobotMap.ahrs.zeroYaw();
		CommandGroup group = new CommandGroup();
		group.addSequential(new DriveTo(path, 150));
		group.addSequential(new ResetDriveTrain());

		Scheduler.getInstance().add(group);

		// RobotMap.updateConstants();
		// UsbCamera usb = CameraServer.getInstance().startAutomaticCapture(0);
		// usb.setExposureManual(0);
		// usb.setBrightness(0);

		isFinished = false;
		RobotMap.driveTrain.zeroEncoders();
		RobotMap.driveTrain.updateConstants();
	}

	private boolean isFinished;

	Mat newMat = new Mat();

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		if (!isFinished) {

		} else {
			RobotMap.driveTrain.reset();
		}

		// RobotMap.driveTrain.setLeftRightVelocity(40, 40);

		// RobotMap.driveTrain.setCurrentModeLeft(false);
		// RobotMap.driveTrain.setCurrentModeRight(false);
		//
		// RobotMap.driveTrainLeft.set(1);
		// RobotMap.driveTrainRight.set(1);

	}

	public void teleopInit() {
		// RobotMap.driveTrain.setLeftRightVelocity(-20, -20);
		ConstantsIO.init();
		RobotMap.updateConstants();
		RobotMap.driveTrain.zeroEncoders();
		temp = 0;
		lastTime = System.currentTimeMillis() / 1000.0;
	}

	double temp = 0;
	double lastTime = 0;

	public void teleopPeriodic() {

		Scheduler.getInstance().run();

		updateSmartDashboard();

		// RobotMap.driveTrain.setCurrentModeLeft(true);
		// RobotMap.driveTrain.setCurrentModeRight(true);
		// RobotMap.driveTrain.enableCurrentMode(true);

		// if (System.currentTimeMillis() % 10000 < 5000) {
		// RobotMap.driveTrainRight.set(4);
		// RobotMap.driveTrainLeft.set(4);
		// } else {
		// RobotMap.driveTrainRight.set(8);
		// RobotMap.driveTrainLeft.set(8);

		// }

		// int curRent =
		// (int)(edu.wpi.first.wpilibj.Timer.getFPGATimestamp()%2)+1;
		// RobotMap.driveTrainLeft.set(curRent);
		// RobotMap.driveTrainRight.set(curRent);
		// RobotMap.driveTrain.setLeftRightVelocity(10, 10);
		double thisTime = System.currentTimeMillis() / 1000.0;
		temp += (thisTime - lastTime) * RobotMap.driveEncRight.getRate();
	}

	public void testInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();
	}

	public void testPeriodic() {

		// if (RobotMap.pressureSwitch.get()) {
		// RobotMap.compressorSpike.set(Relay.Value.kOff);
		// } else {
		// RobotMap.compressorSpike.set(Relay.Value.kForward);
		// }

		updateSmartDashboard();

	}

	public void updateSmartDashboard() {
		// SmartDashboard.putNumber("avg error",
		// RobotMap.driveTrain.getLeftVelocityAvgError());
		SmartDashboard.putNumber("Left Current", RobotMap.driveLeft2.getOutputCurrent());
		SmartDashboard.putNumber("Right Current", RobotMap.driveRight2.getOutputCurrent());
		SmartDashboard.putNumber("Curvature Error", RobotMap.driveTrain.getCurvatureError());
		SmartDashboard.putNumber("Steering", RobotMap.driveTrain.getSteering());
		SmartDashboard.putNumber("Left Velocity Error", RobotMap.driveTrain.getLeftVelocityPIDError());
		SmartDashboard.putBoolean("Test Boolean", System.currentTimeMillis() % 1000 > 500);
		SmartDashboard.putNumber("Test Number", System.currentTimeMillis() % 1000);
		SmartDashboard.putNumber("LeftVelocity", RobotMap.driveEncRateLeft.pidGet());
		SmartDashboard.putNumber("RightVelocity", RobotMap.driveEncRateRight.pidGet());
		SmartDashboard.putNumber("Dist", RobotMap.averageEncoderDistance.pidGet());
		SmartDashboard.putNumber("Angle", RobotMap.ahrs.getAngle());
	}
}
