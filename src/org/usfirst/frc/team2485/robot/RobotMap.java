package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
//	public static Relay compressorSpike;
//	public static DigitalInput pressureSwitch;
	
	public static DriveTrain driveTrain;
	
	
	
	public static void init() {

//		compressorSpike = new Relay(0);
//		pressureSwitch = new DigitalInput(10);
		
		driveTrain = new DriveTrain();
		
	}

	public static void updateConstants() {

	}

}
