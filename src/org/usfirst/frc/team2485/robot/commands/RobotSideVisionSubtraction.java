package org.usfirst.frc.team2485.robot.commands;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Nicholas Contreras
 * @author Sahana Kumar
 */

public class RobotSideVisionSubtraction extends Command {
	private static final int SLEEP_TIME = 5;
	private Thread subtraction;
	private boolean stop;

	public RobotSideVisionSubtraction() {
		setInterruptible(true);
		subtraction = new Thread(() -> processImages());
		subtraction.setDaemon(true);
		stop = false;
	}

	@Override
	protected void initialize() {
		subtraction.start();
	}

	private void processImages() {
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(640, 480);

		CvSink cvSink = CameraServer.getInstance().getVideo();
		CvSource outputStream = CameraServer.getInstance().putVideo("Subtracted Image", 640, 480);

		Mat light = new Mat();
		Mat dark = new Mat();
		Mat output = new Mat();
		RobotMap.lightSpike.set(Value.kForward);
		try {
			Thread.sleep(SLEEP_TIME);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		while (!stop) {
			cvSink.grabFrame(light);
			RobotMap.lightSpike.set(Value.kOff);
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			cvSink.grabFrame(dark);
			RobotMap.lightSpike.set(Value.kForward);
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			Core.subtract(light, dark, output);
			outputStream.putFrame(output);
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		stop = true;
	}
}
