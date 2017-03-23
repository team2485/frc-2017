package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.subsystems.Climber;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.subsystems.Feeder;
import org.usfirst.frc.team2485.subsystems.GearHolder;
import org.usfirst.frc.team2485.subsystems.IntakeArm;
import org.usfirst.frc.team2485.subsystems.IntakeRollers;
import org.usfirst.frc.team2485.subsystems.Shooter;
import org.usfirst.frc.team2485.subsystems.WheelOfDeath;
import org.usfirst.frc.team2485.util.AHRSWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.AHRSWrapperRateAndAngle.Units;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.LidarWrapper;
import org.usfirst.frc.team2485.util.MultipleEncoderWrapper;
import org.usfirst.frc.team2485.util.MultipleEncoderWrapper.MultipleEncoderWrapperMode;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort.Port;
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

	// Physical Constants
	public static final double ROBOT_WIDTH = 27;
	public static final double WHEEL_RADIUS = 2;
	public static final double FEEDER_RADIUS = 0.5;

	// CAN
	public static int driveRightPortCIM1 = 1;
	public static int driveRightPortCIM2 = 3;
	public static int driveRightPortCIM3 = 2;
	public static int driveLeftPortCIM1 = 4;
	public static int driveLeftPortCIM2 = 6;
	public static int driveLeftPortCIM3 = 5;
	public static int wheelOfDeathMotorPort = 7;
	
	// Other Motor Ports
	public static int kShooterMotorPort1 = 8, kShooterMotorPort2 = 9;
	public static int kIntakeMotorPort = 3;
	public static int kFeederMotorPort = 6;
	public static int kClimberMotorPort = 7;
	
	// Sensor Ports
	public static int kRightDriveEncPortA = 0, kRightDriveEncPortB = 1;
	public static int kLeftDriveEncPortA = 3, kLeftDriveEncPortB = 2;
	public static int kFeederEncPortA = 8, kFeederEncPortB = 9; //on competition was 6 and 7
	public static int kShooterEncPortA = 6, kShooterEncPortB = 7; //on competition was 8 and 9
	public static int kUltrasonicPortA = 14, kUltrasonicPortB = 15;

	// Speed Controllers
	public static CANTalon driveLeft1, driveLeft2, driveLeft3, driveRight1, driveRight2, driveRight3, deathMotor;
	public static SpeedControllerWrapper driveTrainRight, driveTrainLeft;
	public static SpeedControllerWrapper shooterMotors, intakeMotor, feederMotor, climberMotor;
	
	// Relays
	public static Relay lightSpike;

	// Compressor
	public static Compressor compressor;

	// Solenoids
	public static Solenoid gearSolenoidWings;
	public static Solenoid gearSolenoidFlaps;
	public static Solenoid intakeArmSolenoidHorizontal;
	public static Solenoid intakeArmSolenoidVertical1;
	public static Solenoid intakeArmSolenoidVertical2;

	// Sensors
	public static Encoder driveEncLeft, driveEncRight;
	public static Encoder shooterEncoder;
	public static Encoder feederEncoder;
	public static Ultrasonic gearDetector;
	public static AHRS ahrs;
//	public static LidarWrapper lidar;
	
	public static MultipleEncoderWrapper averageEncoderDistance, averageEncoderRate;
	public static EncoderWrapperRateAndDistance driveEncRateLeft, driveEncRateRight;
	public static AHRSWrapperRateAndAngle ahrsRateRads;
	
	// Cameras
	public static UsbCamera gearCamera, boilerCamera;

	// Subsystems
	public static GearHolder gearHolder;
	public static DriveTrain driveTrain;
	public static Shooter shooter;
	public static IntakeRollers intakeRollers;
	public static IntakeArm intakeArm;
	public static Climber climber;
	public static WheelOfDeath wheelOfDeath;
	public static Feeder feeder;

	public static void init() {

		// CONSTRUCT HARDWARE
		
		compressor = new Compressor();

		// ACTUATORS
		driveLeft1 = new CANTalon(driveLeftPortCIM1);
		driveLeft2 = new CANTalon(driveLeftPortCIM2);
		driveLeft3 = new CANTalon(driveLeftPortCIM3);

		driveRight1 = new CANTalon(driveRightPortCIM1);
		driveRight2 = new CANTalon(driveRightPortCIM2);
		driveRight3 = new CANTalon(driveRightPortCIM3);

		driveTrainLeft = new SpeedControllerWrapper(driveLeft1, driveLeft2, driveLeft3);
		driveTrainRight = new SpeedControllerWrapper(driveRight1, driveRight2, driveRight3);

		deathMotor = new CANTalon(wheelOfDeathMotorPort);
		deathMotor.setInverted(true);
		deathMotor.changeControlMode(TalonControlMode.PercentVbus);

		shooterMotors = new SpeedControllerWrapper(new VictorSP(kShooterMotorPort1), new VictorSP(kShooterMotorPort2));
		intakeMotor = new SpeedControllerWrapper(new VictorSP(kIntakeMotorPort));
		feederMotor = new SpeedControllerWrapper(new VictorSP(kFeederMotorPort));
		climberMotor = new SpeedControllerWrapper(new VictorSP(kClimberMotorPort));

		lightSpike = new Relay(0);

		gearSolenoidWings = new Solenoid(1);
		gearSolenoidFlaps = new Solenoid(0);
		intakeArmSolenoidHorizontal = new Solenoid(2);
		intakeArmSolenoidVertical1 = new Solenoid(5);
		intakeArmSolenoidVertical2 = new Solenoid(3);

		// SENSORS
		ahrs = new AHRS(Port.kMXP);

		shooterEncoder = new Encoder(kShooterEncPortA, kShooterEncPortB);
		feederEncoder = new Encoder(kFeederEncPortA, kFeederEncPortB);
		
		driveEncLeft = new Encoder(kLeftDriveEncPortA, kLeftDriveEncPortB);
		driveEncRight = new Encoder(kRightDriveEncPortA, kRightDriveEncPortB);

		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);

		gearDetector = new Ultrasonic(14, 15);
		gearDetector.setAutomaticMode(true);
		
//		lidar = new LidarWrapper(edu.wpi.first.wpilibj.I2C.Port.kOnboard);
		
//		gearCamera = CameraServer.getInstance().startAutomaticCapture(0);
//		gearCamera.setResolution(320, 240);
//		gearCamera.setFPS(30);
		
//		boilerCamera = CameraServer.getInstance().startAutomaticCapture(1);
//		boilerCamera.setResolution(640, 480);
		
		averageEncoderDistance = new MultipleEncoderWrapper(PIDSourceType.kDisplacement,
				MultipleEncoderWrapperMode.AVERAGE, driveEncLeft, driveEncRight);

		averageEncoderRate = new MultipleEncoderWrapper(PIDSourceType.kRate, MultipleEncoderWrapperMode.AVERAGE,
				driveEncLeft, driveEncRight);
		ahrsRateRads = new AHRSWrapperRateAndAngle(PIDSourceType.kRate, Units.RADS);

		// CONFIGURE HARDWARE

		driveTrainLeft.setInverted(true);

		driveEncLeft.setDistancePerPulse((double) 1 / 250 * (Math.PI * WHEEL_RADIUS * 2));
		driveEncRight.setDistancePerPulse((double) 1 / 250 * (Math.PI * WHEEL_RADIUS * 2));
		
		shooterEncoder.setDistancePerPulse(1/250.0);
		shooterEncoder.setPIDSourceType(PIDSourceType.kRate);
		
		feederEncoder.setDistancePerPulse((1.0/250)*Math.PI*2*FEEDER_RADIUS);
		feederEncoder.setPIDSourceType(PIDSourceType.kRate);


		// CONSTRUCT SUBSYSTEMS

		driveTrain = new DriveTrain();
		gearHolder = new GearHolder();
		intakeRollers = new IntakeRollers();
		intakeArm = new IntakeArm();
		feeder = new Feeder();
		shooter = new Shooter();
		climber = new Climber();
		wheelOfDeath = new WheelOfDeath();

	}

	public static void updateConstants() {
		wheelOfDeath.updateConstants();
		driveTrain.updateConstants();
		shooter.updateConstants();
		feeder.updateConstants();

	}
}
