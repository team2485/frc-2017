package org.usfirst.frc.team2485.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Scanner;

import org.usfirst.frc.team2485.robot.Robot;

/**
 * Static class to interface IO between the RoboRio and the Driver Station. Used
 * to save constants to a file rather than being hard coded.
 * 
 * @author Ben Clark
 * @author Jeremy McCulloch
 */
public class ConstantsIO {
	public static final String ROBO_RIO_CONSTANTS_FILE_PATH = "/home/lvuser/constants.txt";

	public static HashMap<String, String> data;

	public static double kP_Shooter;
	public static double kI_Shooter;
	public static double kD_Shooter;
	public static double kF_Shooter;
	public static double kShotRPS;
	
	public static double kP_Feeder;
	public static double kI_Feeder;
	public static double kD_Feeder;
	public static double kF_Feeder;
	public static double kFeederRPS;
	
	public static double kP_SWODCurrent;
	public static double kI_SWODCurrent;
	public static double kD_SWODCurrent;
	public static double kF_SWODCurrent;
	public static double kSWODCurrent;
	public static double kSWODMaxVolts;

	public static double kIntakeRollerSpeed;
	
	public static double kClimberSpeed;

	public static double kP_DriveVelocity;
	public static double kI_DriveVelocity;
	public static double kD_DriveVelocity;
	public static double kF_DriveVelocity;

	public static double kP_DriveCurrentCIM;
	public static double kI_DriveCurrentCIM;
	public static double kD_DriveCurrentCIM;
	public static double kF_DriveCurrentCIM;
	
	public static double kP_DriveCurrent775;
	public static double kI_DriveCurrent775;
	public static double kD_DriveCurrent775;
	public static double kF_DriveCurrent775;
	
	public static double kP_Distance;
	public static double kI_Distance;
	public static double kD_Distance;
	public static double kF_Distance;
	
	public static double kDownRamp_OverallVelocity;
	public static double kUpRamp_OverallVelocity;
	public static double kDownRamp_IndividualVelocity;
	public static double kUpRamp_IndividualVelocity;

	public static double kP_DriveSteering;
	public static double kI_DriveSteering;
	public static double kD_DriveSteering;
	public static double kF_DriveSteering;

	public static double kUpRamp_DriveThrottle;
	public static double kDownRamp_DriveThrottle;
	
	public static double kP_DriveAngle;
	public static double kI_DriveAngle;
	public static double kD_DriveAngle;
	
	public static double kP_RotateTo;
	public static double kI_RotateTo;
	public static double kD_RotateTo;
	public static double kF_RotateTo;

	public static double kUpRamp_DriveSteering;
	public static double kDownRamp_DriveSteering;

	public static double kP_DriveAngVel;
	public static double kI_DriveAngVel;
	public static double kD_DriveAngVel;
	public static double kF_DriveAngVel;



	public static void init() {


		System.out.println("ConstantsIO .class file loc: " + ConstantsIO.class.getResource("").getPath());

		if (Robot.isSimulation()) {

			String constantsFile = findConstantsFile();

			try {
				data = parseLoadFile(readLocalFile(constantsFile));
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		} else {
			try {
				data = parseLoadFile(readLocalFile(ROBO_RIO_CONSTANTS_FILE_PATH));
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		}

		//		createUnMatchedConstants();
		kP_Shooter = Double.parseDouble(data.get("kP_Shooter"));
		kI_Shooter = Double.parseDouble(data.get("kI_Shooter"));
		kD_Shooter = Double.parseDouble(data.get("kD_Shooter"));
		kF_Shooter = Double.parseDouble(data.get("kF_Shooter"));
		kShotRPS = Double.parseDouble(data.get("kShotRPS"));
		
		kP_Feeder = Double.parseDouble(data.get("kP_Feeder"));
		kI_Feeder = Double.parseDouble(data.get("kI_Feeder"));
		kD_Feeder = Double.parseDouble(data.get("kD_Feeder"));
		kF_Feeder = Double.parseDouble(data.get("kF_Feeder"));
		kFeederRPS = Double.parseDouble(data.get("kFeederRPS"));

		kP_SWODCurrent = Double.parseDouble(data.get("kP_SWODCurrent"));
		kI_SWODCurrent = Double.parseDouble(data.get("kI_SWODCurrent"));
		kD_SWODCurrent = Double.parseDouble(data.get("kD_SWODCurrent"));
		kF_SWODCurrent = Double.parseDouble(data.get("kF_SWODCurrent"));
		kSWODCurrent = Double.parseDouble(data.get("kSWODCurrent"));
		kSWODMaxVolts = Double.parseDouble(data.get("kSWODMaxVolts"));
		
		kIntakeRollerSpeed = Double.parseDouble(data.get("kIntakeRollerSpeed"));
	
		kClimberSpeed = Double.parseDouble(data.get("kClimberSpeed"));

		kP_DriveVelocity = Double.parseDouble(data.get("kP_DriveVelocity"));
		kI_DriveVelocity = Double.parseDouble(data.get("kI_DriveVelocity"));
		kD_DriveVelocity = Double.parseDouble(data.get("kD_DriveVelocity"));
		kF_DriveVelocity = Double.parseDouble(data.get("kF_DriveVelocity"));

		kP_DriveCurrentCIM = Double.parseDouble(data.get("kP_DriveCurrentCIM"));
		kI_DriveCurrentCIM = Double.parseDouble(data.get("kI_DriveCurrentCIM"));
		kD_DriveCurrentCIM = Double.parseDouble(data.get("kD_DriveCurrentCIM"));
		kF_DriveCurrentCIM = Double.parseDouble(data.get("kF_DriveCurrentCIM"));
		
		kP_DriveCurrent775 = Double.parseDouble(data.get("kP_DriveCurrent775"));
		kI_DriveCurrent775 = Double.parseDouble(data.get("kI_DriveCurrent775"));
		kD_DriveCurrent775 = Double.parseDouble(data.get("kD_DriveCurrent775"));
		kF_DriveCurrent775 = Double.parseDouble(data.get("kF_DriveCurrent775"));


		kP_Distance = Double.parseDouble(data.get("kP_Distance"));
		kI_Distance = Double.parseDouble(data.get("kI_Distance"));
		kD_Distance = Double.parseDouble(data.get("kD_Distance"));
		kF_Distance = Double.parseDouble(data.get("kF_Distance"));

		kUpRamp_DriveThrottle = Double.parseDouble(data.get("kUpRamp_DriveThrottle"));
		kDownRamp_DriveThrottle = Double.parseDouble(data.get("kDownRamp_DriveThrottle"));

		kDownRamp_OverallVelocity = Double.parseDouble(data.get("kDownRamp_OverallVelocity"));
		kUpRamp_OverallVelocity = Double.parseDouble(data.get("kUpRamp_OverallVelocity"));

		kDownRamp_IndividualVelocity = Double.parseDouble(data.get("kDownRamp_IndividualVelocity"));
		kUpRamp_IndividualVelocity = Double.parseDouble(data.get("kUpRamp_IndividualVelocity"));

		kP_DriveSteering = Double.parseDouble(data.get("kP_DriveSteering"));

		kI_DriveSteering = Double.parseDouble(data.get("kI_DriveSteering"));

		kD_DriveSteering = Double.parseDouble(data.get("kD_DriveSteering"));

		kF_DriveSteering = Double.parseDouble(data.get("kF_DriveSteering"));

		kP_DriveAngle = Double.parseDouble(data.get("kP_DriveAngle"));

		kI_DriveAngle = Double.parseDouble(data.get("kI_DriveAngle"));

		kD_DriveAngle = Double.parseDouble(data.get("kD_DriveAngle"));
		
		kP_RotateTo = Double.parseDouble(data.get("kP_RotateTo"));
		
		kI_RotateTo = Double.parseDouble(data.get("kI_RotateTo"));
		
		kD_RotateTo = Double.parseDouble(data.get("kD_RotateTo"));

		kF_RotateTo = Double.parseDouble(data.get("kF_RotateTo"));
		
		kP_DriveAngVel = Double.parseDouble(data.get("kP_DriveAngVel"));
		kI_DriveAngVel = Double.parseDouble(data.get("kI_DriveAngVel"));
		kD_DriveAngVel = Double.parseDouble(data.get("kD_DriveAngVel"));
		kF_DriveAngVel = Double.parseDouble(data.get("kF_DriveAngVel"));

		
		kUpRamp_DriveSteering = Double.parseDouble(data.get("kUpRamp_DriveSteering"));
		kDownRamp_DriveSteering = Double.parseDouble(data.get("kDownRamp_DriveSteering"));
		
	}

	@SuppressWarnings("unused")
	private static void createUnMatchedConstants() {
		Field[] fields = ConstantsIO.class.getDeclaredFields();

		for (int i = 0; i < fields.length; i++) {
			fields[i].getName().startsWith("k");

			if (!data.containsKey(fields[i].getName())) {

			}
		}
	}

	/**
	 * I'm so sorry Jeremy
	 */
	private static String findConstantsFile() {
		File curFolder = new File(ConstantsIO.class.getResource("").getPath());

		while (!curFolder.getName().equals("frc-2017")) {
			curFolder = curFolder.getParentFile();
			System.out.println("Backed out to: " + curFolder.getPath());
		}

		return new File(curFolder, "Constants.txt").getPath().substring(5);
	}

	/**
	 * Used to read a file locally.
	 * 
	 * @param filePath
	 */
	public static String readLocalFile(String filePath) throws IOException {
		File file = new File(filePath);

		System.out.println("Resolved file path: " + file.getPath());

		String fileString;

		StringBuilder fileContents = new StringBuilder((int) file.length());
		Scanner scanner = new Scanner(file);
		String lineSeperator = "\n";

		try {
			while (scanner.hasNextLine())
				fileContents.append(scanner.nextLine() + lineSeperator);
			fileString = fileContents.toString();
			// remove the added "\n"
			fileString = fileString.substring(0, fileString.length() - 1);
		} finally {
			scanner.close();
		}
		return fileString;
	}

	/**
	 * @param loadFileContents
	 * @return HashMap containing constant names and their values as declared in
	 *         the load file.
	 */
	public static HashMap<String, String> parseLoadFile(String fileContents) {

		HashMap<String, String> constantsMap = new HashMap<String, String>();
		Scanner scanner = new Scanner(fileContents);

		while (scanner.hasNextLine()) {

			String currLine = scanner.nextLine().trim();

			if (currLine.contains("=")) {

				String constantName = currLine.substring(0, currLine.indexOf("=")).trim();
				String constantValue = currLine.substring(currLine.indexOf("=") + 1).trim();

				constantsMap.put(constantName, constantValue);
			}
		}
		scanner.close();
		return constantsMap;
	}

	/**
	 * NEEDS TO BE WRITTEN AND DEPLOTED FROM ELSE WHERE: WIDGITS?
	 */
	public static void writeConstantsToRoboRio(String loadFileContents) {

		PrintWriter printWriter = null;

		try {
			printWriter = new PrintWriter(
					new FileOutputStream("ftp://roborio-2485-frc.local" + ROBO_RIO_CONSTANTS_FILE_PATH)); // definitely
			// won't
			// work
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}

		if (printWriter != null) {
			printWriter.write(loadFileContents);
			printWriter.flush();
			printWriter.close();
		} else {
			System.err.println("PrintWriting failed to init, unable to write constants.");
		}

	}

}
