package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * @author Ben Dorsey
 */

public class FlashLights extends Command{
	
	private static final int PERIOD = 2000;
	private ITable table;
//	private Mat uselessMat;
//	private CvSource imageSource;
//	private MjpegServer cvStream;
//	private CvSink imageSink;
	public FlashLights() {
		setInterruptible(true);
		table = NetworkTable.getTable("tableyTable");
//		uselessMat = new Mat(new Size(new double[]{160, 120}), CvType.CV_8U);
//		imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 160, 120, 30);
//		cvStream = new MjpegServer("CV Image Stream", 1185);
//		cvStream.setSource(imageSource);
//		imageSink = new CvSink("CV Image Grabber");
//		imageSink.setSource(RobotMap.usbCam);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
	
	
	@Override
	protected void execute() {
		long curTime = System.currentTimeMillis();
//		curTime = imageSink.grabFrame(uselessMat) / 10000;
		if (curTime % PERIOD < (PERIOD / 2.0)) {
			RobotMap.lightSpike.set(Value.kForward);
			table.putBoolean("on", true);
		} else {
			RobotMap.lightSpike.set(Value.kOff);
			table.putBoolean("on", false);
		}
//		System.out.println(curTime);
//		uselessMat.put(0, 0, curTime % PERIOD);
//		imageSource.putFrame(uselessMat);
//		CameraServer.getInstance().addCamera(imageSource);
	}
	
	@Override
	protected void interrupted() {
		end();
	}
	
	@Override
	protected void end() {
		RobotMap.lightSpike.set(Value.kOff);
	}

}
