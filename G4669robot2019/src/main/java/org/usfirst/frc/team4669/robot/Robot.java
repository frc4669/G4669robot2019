/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot;

import org.usfirst.frc.team4669.robot.commands.*;
import org.usfirst.frc.team4669.robot.subsystems.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static NetworkTableInstance networkTableInst;
	public static NetworkTable visionTable;
	public static OI oi;
	public static F310 f310;
	public static DriverStation driverStation;
	public static DriveTrain driveTrain;
	// public static CubeIntake intake;
	// public static Climber climber;
	public static ElevatorClimber elevator;
	public static Arm arm;
	public static Grabber grabber;
	public AnalogUltrasonic ultrasonic;

	Command autonomousCommand;
	SendableChooser<String> chooser = new SendableChooser<String>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		networkTableInst = NetworkTableInstance.getDefault();
		visionTable = networkTableInst.getTable("DataTable");
		elevator = new ElevatorClimber();
		driveTrain = new DriveTrain();
		arm = new Arm();
		grabber = new Grabber();
		// ultrasonic = new AnalogUltrasonic(0);
		oi = new OI();
		f310 = new F310();
		driveTrain.zeroEncoders();
		driveTrain.resetGyro();
		driveTrain.calibrateGyro();

		// Sends Strings to chooser and not commands in the case of the command
		// requiring something that only occurs during auto init
		chooser.addDefault("Do Nothing", "DoNothing");
		chooser.addObject("Pathfinder", "Pathfinder");
		SmartDashboard.putData("Auto mode", chooser);

	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		driverStation = DriverStation.getInstance();
		updateSmartDashboard();
		if (autonomousCommand != null)
			autonomousCommand.cancel();

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector","Default");
		 * switch(autoSelected) { case "My Auto": autonomousCommand = new
		 * MyAutoCommand(); break; case "Default Auto": default:autonomousCommand = new
		 * ExampleCommand(); break; }
		 */

		if (chooser.getSelected().equals("DoNothing"))
			autonomousCommand = new DoNothing();
		if (chooser.getSelected().equals("Pathfinder"))
			autonomousCommand = new PathfinderTest();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		testSmartDashboard();

	}

	@Override
	public void testInit() {
		SmartDashboard.putNumber("Target Shoulder", 0);
		SmartDashboard.putNumber("Target Elbow", 0);
		SmartDashboard.putNumber("Target Wrist", 0);

		SmartDashboard.putNumber("Target X", 0);
		SmartDashboard.putNumber("Target Y", 0);
		SmartDashboard.putBoolean("Flip Elbow", false);

		// SmartDashboard.putNumber("Target Right Elevator Vel", 0);
		// SmartDashboard.putNumber("Target Left Elevator Vel", 0);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		Scheduler.getInstance().run();
		testSmartDashboard();

	}

	public void updateSmartDashboard() {
		SmartDashboard.putData((Sendable) driveTrain.getGyro());
		SmartDashboard.putData(driveTrain.getGyroController());
	}

	public void testSmartDashboardInit() {
		SmartDashboard.putNumber("Target Shoulder", 0);
		SmartDashboard.putNumber("Target Elbow", 0);
		SmartDashboard.putNumber("Target Wrist", 0);

		SmartDashboard.putNumber("Target X", 0);
		SmartDashboard.putNumber("Target Y", 0);
		SmartDashboard.putBoolean("Flip Elbow", false);

		SmartDashboard.putNumber("Drive Position", 0);
		SmartDashboard.putNumber("Strafe Position", 0);
	}

	public void testSmartDashboard() {
		// SmartDashboard.putNumber("Gyro Angle", driveTrain.getAngle());
		// SmartDashboard.putNumber("Vision Turn Error",
		// driveTrain.getPIDError(driveTrain.getVisionTurnController()));
		// SmartDashboard.putNumber("Vision Distance Error",
		// driveTrain.getPIDError(driveTrain.getVisionDistanceController()));
		// SmartDashboard.putData("Gyro PID Controller", driveTrain.gyroPID);
		// SmartDashboard.putData("Vision Turn PID Controller",
		// driveTrain.getVisionTurnController());
		// SmartDashboard.putData("Vision Distance PID Controller",
		// driveTrain.getVisionDistanceController());
		// SmartDashboard.putData("Align to Ball", new AlignToBall());
		// SmartDashboard.putNumber("Left Encoder", driveTrain.getLeftEncoder());
		// SmartDashboard.putNumber("Right Encoder",
		// Robot.driveTrain.getRightEncoder());
		/**
		 * SmartDashboard.putNumber("Shoulder Position",
		 * arm.getEncoderPosition(arm.getShoulderMotor()));
		 * SmartDashboard.putNumber("Shoulder Velocity",
		 * arm.getEncoderVelocity(arm.getShoulderMotor()));
		 * SmartDashboard.putNumber("Elbow Position",
		 * arm.getEncoderPosition(arm.getElbowMotor())); SmartDashboard.putNumber("Elbow
		 * Velocity", arm.getEncoderVelocity(arm.getElbowMotor()));
		 * SmartDashboard.putNumber("Wrist Position",
		 * arm.getEncoderPosition(arm.getWristMotor())); SmartDashboard.putNumber("Wrist
		 * Velocity", arm.getEncoderVelocity(arm.getWristMotor()));
		 * SmartDashboard.putNumber("Shoulder Angle",
		 * arm.getMotorAngle(arm.getShoulderMotor())); SmartDashboard.putNumber("Elbow
		 * Angle", arm.getMotorAngle(arm.getElbowMotor()));
		 * SmartDashboard.putNumber("Wrist Angle",
		 * arm.getMotorAngle(arm.getWristMotor())); double targetShoulder =
		 * SmartDashboard.getNumber("Target Shoulder", 0); double targetElbow =
		 * SmartDashboard.getNumber("Target Elbow", 0); double targetWrist =
		 * SmartDashboard.getNumber("Target Wrist", 0);
		 * 
		 * double targetX = SmartDashboard.getNumber("Target X", 0); double targetY =
		 * SmartDashboard.getNumber("Target Y", 50);
		 * 
		 * // SmartDashboard.putData("Start Arm Magic", new
		 * ArmMotionMagic(targetShoulder, // targetElbow, targetWrist));
		 * SmartDashboard.putData("Set Arm Angle", new ArmAngleSet(targetShoulder,
		 * targetElbow, targetWrist)); boolean flipUp = SmartDashboard.getBoolean("Flip
		 * Elbow", false);
		 * 
		 * SmartDashboard.putData("Arm to Position", new ArmToPosition(targetX, targetY,
		 * flipUp)); SmartDashboard.putData("Zero Arm Encoders", new ZeroArmEncoders());
		 * 
		 * SmartDashboard.putNumber("Acceleration X", Robot.elevator.getAccelX());
		 * SmartDashboard.putNumber("Acceleration Y", Robot.elevator.getAccelY());
		 * 
		 * double rightElevatorVel = SmartDashboard.getNumber("Target Right Elevator
		 * Vel", 0); double leftElevatorVel = SmartDashboard.getNumber("Target Left
		 * Elevator Vel", 0);
		 * 
		 * SmartDashboard.putData("Set Elevator Speed", new
		 * SetElevatorVelocity(leftElevatorVel, rightElevatorVel));
		 * SmartDashboard.putNumber("Right Elevator Vel",
		 * Robot.elevator.getEncoderVel(Robot.elevator.getRightMotor()));
		 * SmartDashboard.putNumber("Left Elevator Vel",
		 * Robot.elevator.getEncoderVel(Robot.elevator.getLeftMotor()));
		 */
		// SmartDashboard.putNumber("Ultrasonic voltage", ultrasonic.getVoltage());
		// SmartDashboard.putNumber("Ultrasonic Distance", ultrasonic.getDistance());
	}
}
