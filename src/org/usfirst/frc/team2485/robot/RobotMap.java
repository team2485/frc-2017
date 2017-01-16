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
	
	public static SpeedControllerWrapper leftDrive, rightDrive;
	
	public static CANTalon driveTalonRight, driveTalonLeft;
	
	public static CANTalonCurrentWrapper rightCurrentSensor, leftCurrentSensor;
	
	public static Encoder rightDriveEnc, leftDriveEnc;
	
	public static EncoderWrapperRateAndDistance leftDriveEncRate, rightDriveEncRate;
	
	public static void init() {

//		compressorSpike = new Relay(0);
//		pressureSwitch = new DigitalInput(10);
			
		driveTalonRight = new CANTalon(3);
		driveTalonLeft = new CANTalon(2);
		
		rightCurrentSensor = new CANTalonCurrentWrapper(driveTalonRight);
		leftCurrentSensor = new CANTalonCurrentWrapper(driveTalonLeft);
		
		leftDriveEnc = new Encoder(2, 3);
		leftDriveEncRate = new EncoderWrapperRateAndDistance(leftDriveEnc, PIDSourceType.kRate);
		leftDriveEnc.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		rightDriveEnc = new Encoder(0, 1);
		rightDriveEncRate = new EncoderWrapperRateAndDistance(rightDriveEnc, PIDSourceType.kRate);
		rightDriveEnc.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		
		leftDrive = new SpeedControllerWrapper(new SpeedController[] {driveTalonLeft, new VictorSP(5), new VictorSP(6)});
		leftDrive.setInverted(true);
		rightDrive = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(0), new VictorSP(2), driveTalonRight});
		rightDrive.setInverted(false);
		
		driveTrain = new DriveTrain();

	}

	public static void updateConstants() {
		
		driveTrain.updateConstants();
		
	}
}
