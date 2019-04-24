/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot;

import java.awt.Color;
import java.util.Arrays;

import org.usfirst.frc.team4669.robot.commands.*;
import org.usfirst.frc.team4669.robot.commands.arm.*;
import org.usfirst.frc.team4669.robot.commands.elevator.*;
import org.usfirst.frc.team4669.robot.commands.driveTrain.*;
import org.usfirst.frc.team4669.robot.commands.auto.*;
import org.usfirst.frc.team4669.robot.commands.auto.AlignToLine.Direction;
import org.usfirst.frc.team4669.robot.commands.grabber.*;
import org.usfirst.frc.team4669.robot.misc.ArduinoCommunicator;
import org.usfirst.frc.team4669.robot.misc.Constants;
import org.usfirst.frc.team4669.robot.misc.LineAlignEntries;
import org.usfirst.frc.team4669.robot.misc.VisionEntries;
import org.usfirst.frc.team4669.robot.subsystems.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
	public static ButtonBoard buttonBoard;
	// public static ButtonBoard testButtonBoard;
	public static DriveTrain driveTrain;
	public static LineAlignEntries frontLineEntries;
	public static LineAlignEntries backLineEntries;
	public static VisionEntries visionEntries;
	public static ElevatorClimber elevator;
	public static Arm arm;
	public static Grabber grabber;
	public static AnalogDistanceSensor ultrasonic;
	public static int elevatorVel = 300;
	public static int elevatorAccel = 600;
	// public static ArduinoCommunicator arduinoCommunicator;

	// public static boolean endgameStarted = false;

	public static boolean toggleBallMode = false;
	public static boolean toggleCalibrate = false;


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
		oi = new OI();
		f310 = new F310();
		buttonBoard = new ButtonBoard(RobotMap.buttonBoard);
		// testButtonBoard = new ButtonBoard(RobotMap.testButonBoard);

		// testButtonBoard.button1.whenPressed(new AutoClimbLvl2());
		// testButtonBoard.button2.whenPressed(new PositionCommand(ArmData.hookToBall));
		// testButtonBoard.button4.whenPressed(new StrafeStraight(0.3,true));
		
		// arduinoCommunicator = new ArduinoCommunicator(Constants.arduinoBaudRate);

		buttonBoard.button12.whenPressed(new ExtendBothElevator(0));

		visionEntries = new VisionEntries();
		frontLineEntries = new LineAlignEntries(true);
		backLineEntries = new LineAlignEntries(false);


		ArmData.initData();

		driveTrain.zeroEncoders();
		driveTrain.resetGyro();

		driveTrain.calibrateGyro();

		// Sends Strings to chooser and not commands in the case of the command
		// requiring something that only occurs during auto init
		// chooser.addDefault("Do Nothing", "DoNothing");
		// chooser.addObject("Pathfinder", "Pathfinder");
		chooser.addDefault("Hook Grab", "HookGrab");
		ShuffleboardCompetition.createAuto(chooser);
		ShuffleboardCompetition.initialize();
		// SmartDashboard.putData("Auto mode", chooser);
		testSmartDashboardInit();

	}

	public void robotPeriodic(){
		// updateSmartDashboard();
		ShuffleboardCompetition.update();
		//Calibrate Elbow encoder position upon reaching reverse limit switch
		if(toggleCalibrate){
			if(arm.getElbowMotor().getSensorCollection().isRevLimitSwitchClosed())
				arm.getElbowMotor().setSelectedSensorPosition(Constants.calibrateElbow,RobotMap.pidIdx, Constants.timeout);
			if(arm.getShoulderMotor().getSensorCollection().isFwdLimitSwitchClosed())
				arm.getShoulderMotor().setSelectedSensorPosition(Constants.calibrateShoulder,RobotMap.pidIdx, Constants.timeout);
			if(arm.getWristMotor().getSensorCollection().isRevLimitSwitchClosed())
				arm.getWristMotor().setSelectedSensorPosition(Constants.calibrateWrist,RobotMap.pidIdx, Constants.timeout);
		}
		// if(buttonBoard.getButtonPressed(1)){
			// toggleBallMode = !toggleBallMode;
		// }
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
		// if (chooser.getSelected().equals("DoNothing"))
		// 	autonomousCommand = new DoNothing();
		// if (chooser.getSelected().equals("Pathfinder"))
		// 	autonomousCommand = new PathfinderTest();
		if(chooser.getSelected().equals("HookGrab"))
			autonomousCommand = new StartUp();
		// arduinoCommunicator.sendRGB(255,128,0);
		driveTrain.zeroEncoders();
		driveTrain.resetGyro();
		// driveTrain.calibrateGyro();

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
		if (f310.getDPadPOV() != -1) {
			Command turn = new TurnTo(f310.getDPadPOV());
			turn.start();
		} else if(buttonBoard.getAngle()!=-1){
			Command turn = new TurnTo(buttonBoard.getAngle());
			turn.start();
		}
		if (f310.getButtonPressed(F310.orangeButton)){
			Command ballAlign = new AlignToBall();
			ballAlign.start();
		}
		if(buttonBoard.getButtonPressed(1)){
			new ToggleBallHatch().start();;
		}
		if(buttonBoard.getButtonPressed(2)){
			new PathCommand(0).start();
		}
		if(buttonBoard.getButtonPressed(4)){
			if(!toggleBallMode)
				new PathCommand(1).start();
			else
				new PathCommand(1).start();
		}
		if(buttonBoard.getButtonPressed(5)){
			if(!toggleBallMode)
				new PathCommand(2).start();
			else
				new PathCommand(3).start();
		}
		if(buttonBoard.getButtonPressed(6)){
			if(!toggleBallMode)
				new PathCommand(3).start();
			else
				new PathCommand(4).start();
		}
		if(buttonBoard.getButtonPressed(7)&&toggleBallMode){
			new PathCommand(2).start();
		}
		if(buttonBoard.getButtonPressed(9)&&toggleBallMode){
			new FlickWrist().start();
		}
		// if(buttonBoard.getButtonPressed(10)){
		// 	if(!toggleBallMode)
		// 		new PathCommand(5).start();
		// 	else
		// 		new PathCommand(6).start();
		// }
		// if(buttonBoard.getButtonPressed(11)){
		// 	if(!toggleBallMode)
		// 		new PathCommand(4).start();
		// 	else
		// 		new PathCommand(5).start();
		// }
		// if(buttonBoard.getButtonPressed(12)){
		// 	if(!toggleBallMode)
		// 		new PathCommand(3).start();
		// 	else
		// 		new PathCommand(4).start();
		// }
		if(oi.getExtremePOV()==0){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				ArmData upData = new ArmData(lastPosition.getX(), lastPosition.getXCorrection(), lastPosition.getY()+1, lastPosition.getYCorrection(), lastPosition.getGrabberAngle(), lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(upData, upData).start();
			}
		}
		else if(oi.getExtremePOV()==90){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				ArmData fwdData = new ArmData(lastPosition.getX()+1, lastPosition.getXCorrection(), lastPosition.getY(), lastPosition.getYCorrection(), lastPosition.getGrabberAngle(), lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(fwdData, fwdData).start();
			}
		}
		else if(oi.getExtremePOV()==180){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				ArmData downData = new ArmData(lastPosition.getX(), lastPosition.getXCorrection(), lastPosition.getY()-1, lastPosition.getYCorrection(), lastPosition.getGrabberAngle(), lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(downData, downData).start();
			}
		}
		else if(oi.getExtremePOV()==270){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				ArmData fwdData = new ArmData(lastPosition.getX()-1, lastPosition.getXCorrection(), lastPosition.getY(), lastPosition.getYCorrection(), lastPosition.getGrabberAngle(), lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(fwdData, fwdData).start();
			}
		}
		else if(buttonBoard.getButtonPressed(10)){
			ArmData lastPosition = PositionCommand.lastPosition;	
			if(lastPosition != null){
				double angleTilt = 3;
				if(lastPosition.getX()<0){
					angleTilt = -angleTilt;
				}
				ArmData tiltData = new ArmData(lastPosition.getX(), lastPosition.getXCorrection(), lastPosition.getY(), lastPosition.getYCorrection(), lastPosition.getGrabberAngle()-angleTilt, lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(tiltData, tiltData).start();
			}
		}
		else if(buttonBoard.getButtonPressed(11)){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				double angleTilt = 3;
				if(lastPosition.getX()<0){
					angleTilt = -angleTilt;
				}
				ArmData tiltData = new ArmData(lastPosition.getX(), lastPosition.getXCorrection(), lastPosition.getY(), lastPosition.getYCorrection(), lastPosition.getGrabberAngle()+angleTilt, 
						lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(tiltData, tiltData).start();
			}
		}
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
		//Sends alliance color to Arduino for RGB lighting
		// if(DriverStation.getInstance().getAlliance()==DriverStation.Alliance.Red)
		// 	arduinoCommunicator.sendRGB(Color.red.getRed(),Color.red.getGreen(),Color.red.getBlue());
		// else if(DriverStation.getInstance().getAlliance()==DriverStation.Alliance.Blue)
		// 	arduinoCommunicator.sendRGB(Color.blue.getRed(),Color.blue.getGreen(),Color.blue.getBlue());

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		if (f310.getDPadPOV() != -1) {
			Command turn = new TurnTo(f310.getDPadPOV());
			turn.start();
		} else if(buttonBoard.getAngle()!=-1){
			Command turn = new TurnTo(buttonBoard.getAngle());
			turn.start();
		}
		if (f310.getButtonPressed(F310.orangeButton)){
			Command ballAlign = new AlignToBall();
			ballAlign.start();
		}
		if(buttonBoard.getButtonPressed(1)){
			new ToggleBallHatch().start();;
		}
		if(buttonBoard.getButtonPressed(2)){
			new PathCommand(0).start();
		}
		if(buttonBoard.getButtonPressed(4)){
			if(!toggleBallMode)
				new PathCommand(1).start();
			else
				new PathCommand(1).start();
		}
		if(buttonBoard.getButtonPressed(5)){
			if(!toggleBallMode)
				new PathCommand(2).start();
			else
				new PathCommand(3).start();
		}
		if(buttonBoard.getButtonPressed(6)){
			if(!toggleBallMode)
				new PathCommand(3).start();
			else
				new PathCommand(4).start();
		}
		if(buttonBoard.getButtonPressed(7)&&toggleBallMode){
			new PathCommand(2).start();
		}
		else if(buttonBoard.getButtonPressed(9)&&toggleBallMode){
			new FlickWrist().start();
		}
	
		if(oi.getExtremePOV()==0){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				ArmData upData = new ArmData(lastPosition.getX(), lastPosition.getXCorrection(), lastPosition.getY()+1, lastPosition.getYCorrection(), lastPosition.getGrabberAngle(), lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(upData, upData).start();
			}
		}
		else if(oi.getExtremePOV()==90){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				ArmData fwdData = new ArmData(lastPosition.getX()+1, lastPosition.getXCorrection(), lastPosition.getY(), lastPosition.getYCorrection(), lastPosition.getGrabberAngle(), lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(fwdData, fwdData).start();
			}
		}
		else if(oi.getExtremePOV()==180){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				ArmData downData = new ArmData(lastPosition.getX(), lastPosition.getXCorrection(), lastPosition.getY()-1, lastPosition.getYCorrection(), lastPosition.getGrabberAngle(), lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(downData, downData).start();
			}
		}
		else if(oi.getExtremePOV()==270){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				ArmData fwdData = new ArmData(lastPosition.getX()-1, lastPosition.getXCorrection(), lastPosition.getY(), lastPosition.getYCorrection(), lastPosition.getGrabberAngle(), lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(fwdData, fwdData).start();
			}
		}
		else if(buttonBoard.getButtonPressed(10)){
			ArmData lastPosition = PositionCommand.lastPosition;	
			if(lastPosition != null){
				double angleTilt = 3;
				if(lastPosition.getX()<0){
					angleTilt = -angleTilt;
				}
				ArmData tiltData = new ArmData(lastPosition.getX(), lastPosition.getXCorrection(), lastPosition.getY(), lastPosition.getYCorrection(), lastPosition.getGrabberAngle()-angleTilt, lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(tiltData, tiltData).start();
			}
		}
		else if(buttonBoard.getButtonPressed(11)){
			ArmData lastPosition = PositionCommand.lastPosition;
			if(lastPosition != null){
				double angleTilt = 3;
				if(lastPosition.getX()<0){
					angleTilt = -angleTilt;
				}
				ArmData tiltData = new ArmData(lastPosition.getX(), lastPosition.getXCorrection(), lastPosition.getY(), lastPosition.getYCorrection(), lastPosition.getGrabberAngle()+angleTilt, 
						lastPosition.getAngleCorrection(), lastPosition.getFlip());
				new PositionCommand(tiltData, tiltData).start();
			}
		}

		if (oi.getExtremeRawButtonPressed(8)){
			new AutoClimbLvl3().start();
		}

		if (oi.getExtremeRawButtonPressed(7)){
			new AutoClimbLvl2().start();
		}

		// if(Timer.getMatchTime()<=30&&!endgameStarted){
		// 	// arduinoCommunicator.sendString("endgame");
		// 	endgameStarted = true;
		// }
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

	}

	public void updateSmartDashboard() {
		testSmartDashboard();
	}

	public void testSmartDashboardInit() {
		SmartDashboard.putNumber("Arm/Target Shoulder", 0);
		SmartDashboard.putNumber("Arm/Target Elbow", 0);
		SmartDashboard.putNumber("Arm/Target Wrist", 0);

		SmartDashboard.putNumber("Arm/Target X", 0);
		SmartDashboard.putNumber("Arm/Target Y", 0);
		SmartDashboard.putNumber("Arm/Wrist Angle", 0);

		SmartDashboard.putBoolean("Arm/Flip Wrist", true);
		SmartDashboard.putBoolean("Arm/Flip Elbow", false);

		SmartDashboard.putData("Line Align", new AlignToLine(Direction.FRONT));
		SmartDashboard.putData("Toggle Compressor", new ToggleCompressor());
		SmartDashboard.putData("Arm/Starter Arm", new StarterArm());
		SmartDashboard.putData("Arm/Zero Arm Encoders", new ZeroArmEncoders());


		// SmartDashboard.putNumber("Elevator Distance", 0);
		// SmartDashboard.putNumber("Elevator Drive", 0);

		// SmartDashboard.putNumber("Drive Position", 0);
		// SmartDashboard.putNumber("Strafe distance", 0);
		// SmartDashboard.putNumber("Turn To", 0);
		// SmartDashboard.putNumber("Strafe Position", 0);

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
		SmartDashboard.putNumber("Arm/Elbow Position", arm.getEncoderPosition(arm.getElbowMotor()));
		SmartDashboard.putNumber("Arm/Wrist Position", arm.getEncoderPosition(arm.getWristMotor()));
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
		double wristAngle = SmartDashboard.getNumber("Arm/Wrist Angle", 0);
		boolean flipWrist = SmartDashboard.getBoolean("Arm/Flip Wrist",false);

		SmartDashboard.putData("Arm/Set Arm Angle", new ArmAngleSet(targetShoulder, targetElbow, targetWrist));
		boolean flipUp = SmartDashboard.getBoolean("Arm/Flip Elbow", false);

		// SmartDashboard.putData("Arm/Arm to Position", new ArmToPosition(targetX, 0, targetY, 0, wristAngle, 0, flipUp, flipWrist));

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
