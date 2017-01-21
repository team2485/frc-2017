package org.usfirst.frc.team2485.robot;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.subsystems.GearHolder;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;


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
	public static int driveLeftPort1 = 0;
	public static int driveLeftPort2 = 0;
	public static int driveLeftPort3 = 0;
	public static int driveRightPort1 = 0;
	public static int driveRightPort2 = 0;
	public static int driveRightPort3 = 0;

	

	// speed controllers
//	public static SpeedControllerWrapper driveTrainLeft, driveTrainRight;
	public static CANTalon driveLeft1, driveLeft2, driveLeft3, driveRight1, driveRight2, driveRight3;
	
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
		
		driveLeft1 = new CANTalon(driveLeftPort1);
		driveLeft2 = new CANTalon(driveLeftPort2);
		driveLeft3 = new CANTalon(driveLeftPort3);
		
		driveRight1 = new CANTalon(driveRightPort1);
		driveRight2 = new CANTalon(driveRightPort2);
		driveRight3 = new CANTalon(driveRightPort3);
		
		driveEncLeft = new Encoder(2, 3);
		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
		driveEncRight = new Encoder(0, 1);
		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);
		
//		driveTrainLeft = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(4), new VictorSP(5), new VictorSP(6)});
//		driveTrainRight = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(0), new VictorSP(2), new VictorSP(1)});

		gearSolenoid1 = new Solenoid(0);
		gearSolenoid2 = new Solenoid(1);
						
		
		//configure hardware
		driveLeft1.changeControlMode(TalonControlMode.Current);
		driveLeft2.changeControlMode(TalonControlMode.Follower);
		driveLeft3.changeControlMode(TalonControlMode.Follower);
		
		driveRight1.changeControlMode(TalonControlMode.Current);
		driveRight2.changeControlMode(TalonControlMode.Follower);
		driveRight3.changeControlMode(TalonControlMode.Follower);
		
		driveLeft2.set(driveLeftPort1);
		driveLeft3.set(driveLeftPort1);
		driveRight2.set(driveRightPort1);
		driveRight3.set(driveRightPort1);
		
		driveLeft1.setInverted(true);
		
		driveEncLeft.setReverseDirection(true);
		driveEncRight.setReverseDirection(false);
		driveEncLeft.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		driveEncRight.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		
//		driveTrainLeft.setInverted(true);
//		driveTrainRight.setInverted(false);

		
		//construct subsytems
		driveTrain = new DriveTrain();
		gearHolder = new GearHolder();

	}

	public static void updateConstants() {
		
		driveTrain.updateConstants();
		
	}
}
