package org.usfirst.frc.team2485.robot;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.subsystems.GearHolder;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
//	public static Relay compressorSpike;
//	public static DigitalInput pressureSwitch;
	 
	//constants
	public static final double wheelRadius = 2;
	
	// speed controllers
	public static SpeedControllerWrapper driveTrainLeft, driveTrainRight;
	
	//solenoids
	public static Solenoid gearSolenoid1, gearSolenoid2;

	//sensors
	public static Encoder driveEncLeft, driveEncRight;
	public static EncoderWrapperRateAndDistance driveEncRateLeft, driveEncRateRight;
	
	// subsystems
	public static GearHolder gearHolder;
	public static DriveTrain driveTrain;

	public static void init() {

		
		// construct hardware
//		compressorSpike = new Relay(0);
//		pressureSwitch = new DigitalInput(10);
		
		driveEncLeft = new Encoder(2, 3);
		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
		driveEncRight = new Encoder(0, 1);
		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);
		
		driveTrainLeft = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(4), new VictorSP(5), new VictorSP(6)});
		driveTrainRight = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(0), new VictorSP(2), new VictorSP(1)});
		
		gearSolenoid1 = new Solenoid(0);
		gearSolenoid2 = new Solenoid(1);
						
		
		//configure hardware
		driveEncLeft.setReverseDirection(true);
		driveEncRight.setReverseDirection(false);
		driveEncLeft.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		driveEncRight.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		
		driveTrainLeft.setInverted(true);
		driveTrainRight.setInverted(false);

		
		//construct subsytems
		driveTrain = new DriveTrain();
		gearHolder = new GearHolder();

	}

	public static void updateConstants() {
		
		driveTrain.updateConstants();
		
	}
}
