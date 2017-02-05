package org.usfirst.frc.team2485.robot;
import org.usfirst.frc.team2485.subsystems.Climber;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.subsystems.Feeder;
import org.usfirst.frc.team2485.subsystems.GearHolder;
import org.usfirst.frc.team2485.subsystems.IntakeArm;
import org.usfirst.frc.team2485.subsystems.IntakeRollers;
import org.usfirst.frc.team2485.subsystems.Shooter;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;

import com.ctre.CANTalon;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
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
	public static int driveRightPort1 = 1;
	public static int driveRightPort2 = 4;
	public static int driveRightPort3 = 2;
	public static int driveLeftPort1 = 3;
	public static int driveLeftPort2 = 6;
	public static int driveLeftPort3 = 5;
	public static int kShooterEncoderPortA = 4, kShooterEncoderPortB = 5;
	public static int kLeftDriveEnc1 = 0, kLeftDriveEnc2 = 1;
	public static int kRightDriveEnc1 = 2, kRightDriveEnc2 = 3;
	public static int kShooterMotorPort = 0;
	public static int kIntakeMotorPort = 1;
	public static int kFeederEncoderPortA = 6, kFeederEncoderPortB = 7;
	public static int kFeederMotorPort = 2;
	public static int kClimberMotorPort = 7;
	

	

	// speed controllers
	public static SpeedControllerWrapper driveTrainRight, driveTrainLeft;
	public static CANTalon driveLeft1, driveLeft2, driveLeft3, driveRight1, driveRight2, driveRight3;
	public static SpeedControllerWrapper shooterMotor;		
	public static SpeedControllerWrapper intakeMotor;
	public static SpeedControllerWrapper feederMotor;
	public static SpeedControllerWrapper climberMotor;
	
	//solenoids
	public static Solenoid gearSolenoidBottom1, gearSolenoidBottom2;
	public static Solenoid gearSolenoidTop1, gearSolenoidTop2;
	public static Solenoid intakeArmSolenoid;

	//sensors
	public static Encoder driveEncLeft, driveEncRight;
	public static EncoderWrapperRateAndDistance driveEncRateLeft, driveEncRateRight;
	public static Encoder shooterEncoder;	
	public static Encoder feederEncoder;
	public static Ultrasonic gearDetector;
	
	// subsystems
	public static GearHolder gearHolder;
	public static DriveTrain driveTrain;

	
	public static UsbCamera usbCam;

	public static Shooter shooter;
	public static IntakeRollers intakeRollers;
	public static IntakeArm intakeArm;
	public static Climber climber;
	
	public static Feeder feeder;
	public static Relay lightSpike;
	
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
		
		driveTrainLeft = new SpeedControllerWrapper(driveLeft1, driveLeft2, driveLeft3);
		driveTrainRight = new SpeedControllerWrapper(driveRight1, driveRight2, driveRight3);
		
//		driveEncLeft = new Encoder(2, 3);
//		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
//		driveEncRight = new Encoder(0, 1);
//		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);
		
		shooterMotor = new SpeedControllerWrapper(new VictorSP(kShooterMotorPort));
		intakeMotor = new SpeedControllerWrapper(new VictorSP(kIntakeMotorPort));
		feederMotor = new SpeedControllerWrapper(new VictorSP(kFeederMotorPort));
		climberMotor = new SpeedControllerWrapper(new VictorSP(kClimberMotorPort));
		
		shooterEncoder = new Encoder(kShooterEncoderPortA, kShooterEncoderPortB);
		feederEncoder = new Encoder(kFeederEncoderPortA, kFeederEncoderPortB);
		
		lightSpike = new Relay(0);
		
//		usbCam = new UsbCamera("cam0", 0);
				
		
			

				
//		driveTrainLeft = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(4), new VictorSP(5), new VictorSP(6)});
//		driveTrainRight = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(0), new VictorSP(2), new VictorSP(1)});

		gearSolenoidBottom1 = new Solenoid(0);
		gearSolenoidBottom2 = new Solenoid(1);
		gearSolenoidTop1 = new Solenoid(3);
		gearSolenoidTop2 = new Solenoid(4); 
		intakeArmSolenoid = new Solenoid(2);
						
		
		//configure hardware
//		driveLeft1.changeControlMode(TalonControlMode.Current);
//		driveLeft2.changeControlMode(TalonControlMode.Current);
//		driveLeft3.changeControlMode(TalonControlMode.Current);
//		
//		driveRight1.changeControlMode(TalonControlMode.Current);
//		driveRight2.changeControlMode(TalonControlMode.Current);
//		driveRight3.changeControlMode(TalonControlMode.Current);
		
		driveTrainLeft.setInverted(true);
		
		driveEncLeft = new Encoder(kLeftDriveEnc1, kLeftDriveEnc2);
		driveEncRight = new Encoder(kRightDriveEnc1, kRightDriveEnc2);
		
		driveEncLeft.setReverseDirection(true);
		driveEncRight.setReverseDirection(false);
		driveEncLeft.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		driveEncRight.setDistancePerPulse((double)1/250 * (Math.PI * wheelRadius * 2));
		
		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);
		
		//construct subsytems
		driveTrain = new DriveTrain();
		gearHolder = new GearHolder();
		
		intakeRollers = new IntakeRollers();
		intakeArm = new IntakeArm();
		feeder = new Feeder();
		shooter = new Shooter(); 
		climber = new Climber();
		

	}

	public static void updateConstants() {
		
		driveTrain.updateConstants();
		
	}
}
