package org.usfirst.frc.team2485.robot;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.util.CANTalonCurrentWrapper;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
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
	
	public static CANTalon driveTalonRight, driveTalonLeft;
	
	public static CANTalonCurrentWrapper currrentSensorLeft, currentSensorRight;
	
	public static Encoder driveEncLeft, driveEncRight;
	
	public static EncoderWrapperRateAndDistance driveEncRateLeft, driveEncRateRight;
	
	public static void init() {

//		compressorSpike = new Relay(0);
//		pressureSwitch = new DigitalInput(10);
			
		driveTalonRight = new CANTalon(3);
		driveTalonLeft = new CANTalon(2);
		
		currentSensorRight = new CANTalonCurrentWrapper(driveTalonRight);
		currentSensorRight = new CANTalonCurrentWrapper(driveTalonLeft);
		
		driveEncLeft = new Encoder(2, 3);
		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
		driveEncLeft.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		driveEncRight = new Encoder(0, 1);
		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);
		driveEncRight.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		
		driveTrainLeft = new SpeedControllerWrapper(new SpeedController[] {driveTalonLeft, new VictorSP(5), new VictorSP(6)});
		driveTrainLeft.setInverted(true);
		driveTrainRight = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(0), new VictorSP(2), driveTalonRight});
		driveTrainRight.setInverted(false);
		
		driveTrain = new DriveTrain();

	}

	public static void updateConstants() {
		
		driveTrain.updateConstants();
		
	}
}
