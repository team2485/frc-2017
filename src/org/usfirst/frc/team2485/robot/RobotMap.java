package org.usfirst.frc.team2485.robot;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.subsystems.GearHolder;
import org.usfirst.frc.team2485.util.CANTalonCurrentWrapper;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TalonSRX;
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
	
	public static final double wheelRadius = 2;
	
	public static DriveTrain driveTrain;
	
	public static SpeedControllerWrapper driveTrainLeft, driveTrainRight;
	
	public static Encoder driveEncLeft, driveEncRight;
	
	public static EncoderWrapperRateAndDistance driveEncRateLeft, driveEncRateRight;
	
	public static Solenoid gearSolenoid1, gearSolenoid2;

	public static GearHolder gearHolder;
	
	public static void init() {

//		compressorSpike = new Relay(0);
//		pressureSwitch = new DigitalInput(10);
		
		driveEncLeft = new Encoder(2, 3);
		driveEncLeft.setReverseDirection(true);
		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
		driveEncLeft.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		driveEncRight = new Encoder(0, 1);
		driveEncRight.setReverseDirection(false);
		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);
		driveEncRight.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		
		driveTrainLeft = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(4), new VictorSP(5), new VictorSP(6)});
		driveTrainLeft.setInverted(true);
		driveTrainRight = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(0), new VictorSP(2), new VictorSP(1)});
		driveTrainRight.setInverted(false);
		
		driveTrain = new DriveTrain();
		
		gearSolenoid1 = new Solenoid(0);
		gearSolenoid2 = new Solenoid(1);
		
		gearHolder = new GearHolder();

	}

	public static void updateConstants() {
		
		driveTrain.updateConstants();
		
	}
}
