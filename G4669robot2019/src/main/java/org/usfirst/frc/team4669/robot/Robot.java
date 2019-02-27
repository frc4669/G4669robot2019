/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot;

import org.usfirst.frc.team4669.robot.commands.*;
import org.usfirst.frc.team4669.robot.commands.arm.*;
import org.usfirst.frc.team4669.robot.commands.elevator.*;
import org.usfirst.frc.team4669.robot.commands.driveTrain.*;
import org.usfirst.frc.team4669.robot.commands.auto.*;
import org.usfirst.frc.team4669.robot.commands.auto.AlignToLine.Direction;
import org.usfirst.frc.team4669.robot.commands.grabber.*;
import org.usfirst.frc.team4669.robot.misc.Constants;
import org.usfirst.frc.team4669.robot.misc.LineAlignEntries;
import org.usfirst.frc.team4669.robot.misc.VisionEntries;
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
	public static LineAlignEntries frontLineEntries;
	public static LineAlignEntries backLineEntries;
	public static VisionEntries visionEntries;
	// public static CubeIntake intake;
	// public static Climber climber;
	public static ElevatorClimber elevator;
	public static Arm arm;
	public static Grabber grabber;
	public static PixySubsystem pixySub;
	public static AnalogDistanceSensor ultrasonic;
	public static int elevatorVel = 300;
	public static int elevatorAccel = 600;

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
		pixySub = new PixySubsystem();
		// ultrasonic = new AnalogUltrasonic(0);
		oi = new OI();
		f310 = new F310();

		visionEntries = new VisionEntries();
		frontLineEntries = new LineAlignEntries(true);
		backLineEntries = new LineAlignEntries(false);

		driveTrain.zeroEncoders();
		driveTrain.resetGyro();
		driveTrain.calibrateGyro();

		

		// Sends Strings to chooser and not commands in the case of the command
		// requiring something that only occurs during auto init
		chooser.addDefault("Do Nothing", "DoNothing");
		chooser.addObject("Pathfinder", "Pathfinder");
		SmartDashboard.putData("Auto mode", chooser);
		testSmartDashboardInit();

	}

	public void robotPeriodic(){
		updateSmartDashboard();
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
		// updateSmartDashboard();
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
		// updateSmartDashboard();
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
		if (f310.getDPadPOV() != -1) {
			Command turn = new TurnTo(f310.getDPadPOV());
			turn.start();
		}
		Scheduler.getInstance().run();
		// updateSmartDashboard();
		// testSmartDashboard();

	}

	@Override
	public void testInit() {
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		Scheduler.getInstance().run();
		// testSmartDashboard();

	}

	public void updateSmartDashboard() {
		SmartDashboard.putData((Sendable) driveTrain.getGyro());
		// SmartDashboard.putData(driveTrain.getGyroController());
		testSmartDashboard();
	}

	public void testSmartDashboardInit() {
		SmartDashboard.putNumber("Arm/Target Shoulder", 0);
		SmartDashboard.putNumber("Arm/Target Elbow", 0);
		SmartDashboard.putNumber("Arm/Target Wrist", 0);

		SmartDashboard.putNumber("Arm/Target X", 0);
		SmartDashboard.putNumber("Arm/Target Y", 0);
		SmartDashboard.putBoolean("Arm/Flip Wrist", false);
		SmartDashboard.putBoolean("Arm/Flip Elbow", false);

		SmartDashboard.putData("Line Align", new AlignToLine(Direction.FRONT));
		SmartDashboard.putData("Toggle Compressor", new ToggleCompressor());

		// SmartDashboard.putNumber("Elevator Distance", 0);
		// SmartDashboard.putNumber("Elevator Drive", 0);

		// SmartDashboard.putNumber("Drive Position", 0);
		// SmartDashboard.putNumber("Strafe distance", 0);
		// SmartDashboard.putNumber("Turn To", 0);
		// SmartDashboard.putNumber("Strafe Position", 0);

	}

	public void testSmartDashboard() {
		SmartDashboard.putNumber("Gyro Angle", driveTrain.getAngle());
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
		/*
		 * SmartDashboard.putNumber("Front Left Encoder",
		 * driveTrain.getFrontLeftEncoder());
		 * SmartDashboard.putNumber("Front Right Encoder",
		 * driveTrain.getFrontRightEncoder());
		 * SmartDashboard.putNumber("Front Left Vel",
		 * driveTrain.getFrontLeftEncoderSpeed());
		 * SmartDashboard.putNumber("Front Right Vel",
		 * driveTrain.getFrontRightEncoderSpeed()); double distance =
		 * SmartDashboard.getNumber("Drive Position", 0);
		 * SmartDashboard.putData("Motion Magic Drive", new
		 * DriveForwardMotionMagic(distance));
		 * SmartDashboard.putData("Zero Drive Train Encoders", new
		 * ZeroDriveTrainEncoder()); SmartDashboard.putData("BallAlignment", new
		 * BallAlignment()); double strafe = SmartDashboard.getNumber("Strafe distance",
		 * 0); SmartDashboard.putData("Strafe Motion Magic", new
		 * StrafeMotionMagic(strafe)); double turnAngle =
		 * SmartDashboard.getNumber("Turn To", 0);
		 * SmartDashboard.putData("Turn To Command", new TurnTo(turnAngle));
		 */

		SmartDashboard.putNumber("Arm/Shoulder Position", arm.getEncoderPosition(arm.getShoulderMotor()));
		SmartDashboard.putNumber("Arm/Shoulder Velocity", arm.getEncoderVelocity(arm.getShoulderMotor()));
		SmartDashboard.putNumber("Arm/Elbow Position", arm.getEncoderPosition(arm.getElbowMotor()));
		SmartDashboard.putNumber("Arm/Elbow Velocity", arm.getEncoderVelocity(arm.getElbowMotor()));
		SmartDashboard.putNumber("Arm/Wrist Position", arm.getEncoderPosition(arm.getWristMotor()));
		SmartDashboard.putNumber("Arm/Wrist Velocity", arm.getEncoderVelocity(arm.getWristMotor()));
		SmartDashboard.putNumber("Arm/Shoulder Angle", arm.getMotorAngle(arm.getShoulderMotor()));
		SmartDashboard.putNumber("Arm/Elbow Angle", arm.getMotorAngle(arm.getElbowMotor()));
		SmartDashboard.putNumber("Arm/Wrist Angle", arm.getMotorAngle(arm.getWristMotor()));
		double targetShoulder = // *
				SmartDashboard.getNumber("Arm/Target Shoulder", 0);
		double targetElbow = // *
				SmartDashboard.getNumber("Arm/Target Elbow", 0);
		double targetWrist = // *
				SmartDashboard.getNumber("Arm/Target Wrist", 0);

		double targetX = SmartDashboard.getNumber("Arm/Target X", 20);
		double targetY = SmartDashboard.getNumber("Arm/Target Y", 20);
		double wristAngle = 0;
		if(SmartDashboard.getBoolean("Arm/Flip Wrist",false)){
			wristAngle = 180;
		}

		SmartDashboard.putData("Arm/Set Arm Angle", new ArmAngleSet(targetShoulder, targetElbow, targetWrist));
		boolean flipUp = SmartDashboard.getBoolean("Arm/Flip Elbow", false);

		SmartDashboard.putData("Arm/Arm to Position", new ArmToPosition(targetX, targetY, wristAngle, flipUp));
		SmartDashboard.putData("Arm/Zero Arm Encoders", new ZeroArmEncoders());

		// SmartDashboard.putNumber("Acceleration X", Robot.elevator.getAccelX());
		// SmartDashboard.putNumber("Acceleration Y", Robot.elevator.getAccelY());

		// SmartDashboard.putNumber("Right Elevator Inches",
		// elevator.getEncoderPos(elevator.getRightMotor()) *
		// Constants.encoderToInchElevator);
		// SmartDashboard.putNumber("Left Elevator Inches",
		// elevator.getEncoderPos(elevator.getLeftMotor()) *
		// Constants.encoderToInchElevator);

		// SmartDashboard.putNumber("Wheel Elevator Inches",
		// elevator.getEncoderPos(elevator.getWheelMotor()) *
		// Constants.encoderToInchDrive);

		// double distance = SmartDashboard.getNumber("Elevator Distance", 0);
		// double elevatorDrive = SmartDashboard.getNumber("Elevator Drive", 0);
		// SmartDashboard.putData("Extend Elevators", new ExtendBothElevator(distance));
		// SmartDashboard.putData("Extend Elevators Left", new
		// ExtendLeftElevator(distance));
		// SmartDashboard.putData("Extend Elevators Right", new
		// ExtendRightElevator(distance));

		// SmartDashboard.putData("Drive Elevator Cmd", new
		// DriveElevatorMotionMagic(elevatorDrive));

		// SmartDashboard.putNumber("Right Elevator Vel",
		// Robot.elevator.getEncoderVel(Robot.elevator.getRightMotor()));
		// SmartDashboard.putNumber("Left Elevator Vel",
		// Robot.elevator.getEncoderVel(Robot.elevator.getLeftMotor()));
		// SmartDashboard.putNumber("Wheel Elevator Vel",
		// Robot.elevator.getEncoderVel(Robot.elevator.getWheelMotor()));
		// SmartDashboard.putNumber("Ultrasonic voltage", ultrasonic.getVoltage());
		// SmartDashboard.putNumber("Ultrasonic Distance", ultrasonic.getDistance());
	}

}
