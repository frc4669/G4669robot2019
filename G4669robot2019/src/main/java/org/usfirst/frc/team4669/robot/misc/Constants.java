/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team4669.robot.misc;

import org.usfirst.frc.team4669.robot.Robot;

/**
 * Include field measurements, robot length measurements, and other numbered
 * constants here to reduce magic numbers floating around.
 */
public class Constants {
	// Robot Constants
	public static final double wheelDiameter = 4; // in inches
	public static final double elevatorSprocketDiameter = 1.25; // in inches
	public static final double wheelBase = 22.25; // figure out real distance later
	public static final int encoderTicksPerRotation = 4096;
	public static final double upperArmLength = 17.25;
	public static final double shoulderHeight = 26.25;
	public static final double armHeightOffset = 2;
	public static final double forearmLength = 35;
	public static final double handHookLength = 8.5;
	public static final double handHoopLength = 13;
	public static final double shoulderGearRatio = 4.5;
	public static final double elbowGearRatio = 4.5;
	public static final double wristGearRatio = 2;

	public static final int pixyWidth = 320; // pixels
	public static final int pixyHeight = 200; // pixels
	public static final double pixyFocalLength = 2.8 / 25.4; // 2.8mm to inches
	public static final double pixyFOV = 75; // 75 degrees

	public static final int pixy2LineWidth = 78; // pixels
	public static final int pixy2LineHeight = 51; // pixels

	public static final int ballDiameter = 13;

	// Position Constants, figure out actual numbers later
	public static final double elevatorTolerance = 200; // Units in encoder position
	public static final double level2HeightInches = 6.5; // Rest of the units are inches
	public static final double wheelElevatorOffSet = 2.5;
	public static final double climbDriveDistance = 3;
	public static final double climbDriveDistance2 = 15;
	public static final double climbDriveDistance3 = 5;
	public static final double level3HeightInches = 19.5;
	public static final double incrementalHeightInches = 6.5;
	public static final double limitMaxHeight = 22;

	public static final double xDistanceToPlace = 18;
	public static final double hatch1Height = 18.3;
	public static final double hatch2Height = 46.3;
	public static final double hatch3Height = 74.3;
	public static final double ball1Height = 26.8;
	public static final double ball2Height = 54.8;
	public static final double ball3Height = 82.8;
	public static final double ballShipHeight = 42;
	public static final double robotToArmFront = 20.9;
	public static final double robotToArmBack = 8.5;

	public static final double armGrabBallX = 15;
	public static final double armGrabBallY = 6.5;

	// Constants for Pathfinder
	public static final double maxVel = 77.4; // units in inches

	public static final double strafekP = 1.5;
	public static final double strafekD = 5;

	/**
	 * Array for accessing PID constants for turning drive train
	 * {kF,kP,kI,kD,Integral Zone}
	 */
	public static final double[] driveTrainPID = { 0.4049, 8, 0.96, 160, 50 };

	/** Array for accessing PID constants for the gyro turning {kP,kI,kD} */
	public static final double[] gyroPID = { 0.25, 0, 0.045 };

	/** Array for accessing PID constants for vision {kP,kI,kD} */
	public static final double[] cameraPID = { 0.8, 0, 0 };

	/**
	 * Array for accessing PID constants for elevator {kF,kP,kI,kD,Integral zone}
	 */
	// public static final double[] elevatorPID = { 0.977, 1.056, 0.006, 21.12, 50
	// };
	public static final double[] elevatorPID = { 1.78, 1.8, 0, 50, 50 };

	/** Array for accessing PID constants for arm {kF,kP,kI,kD,Integral zone} */
	public static final double[] shoulderPID = { 1.26, 3, 0, 0, 50 };

	/** Array for accessing PID constants for arm {kF,kP,kI,kD,Integral zone} */
	public static final double[] elbowPID = { 1.81, 3.1, 0.0002, 100, 50 };

	/** Array for accessing PID constants for arm {kF,kP,kI,kD,Integral zone} */
	public static final double[] wristPID = { 1.76, 1.2, 0, 0, 50 };

	/**
	 * Array for accessing PID constants for climber drive {kF,kP,kI,kD,Integral
	 * zone}
	 */
	public static final double[] elevatorWheelPID = { 0.0925, 3, 0.5, 40, 50 };

	public static final int timeout = 10;
	public static final int baseTrajPeriodMs = 0;

	// Velocities & Acceleration for Motion Magic
	public static final int elevatorVel = 450;
	public static final int elevatorAccel = 1200;

	public static final int elevatorWheelVel = 1000;
	public static final int elevatorWheelAccel = 2000;

	public static final int driveVel = 2300;
	public static final int driveAccel = 4600;

	public static final double armScaleFactor = 1;

	public static final int defaultElbowPos = -6089;

	public static final int startShoulder = 5279;
	public static final int startElbow = -7920+615/2;
	public static final int startWrist = -2116+(int)(4*wristGearRatio*encoderTicksPerRotation/360);

	public static final int calibrateShoulder = 7869-(int)(1*shoulderGearRatio*encoderTicksPerRotation/360);
	public static final int calibrateElbow = -7860+615/2;
	public static final int calibrateWrist = -5286+(int)(4*wristGearRatio*encoderTicksPerRotation/360);


	
	public static final int shoulderVel = (int) (200 * armScaleFactor);
	public static final int shoulderAccel = (int) (375 * armScaleFactor);
	public static final int elbowVel = (int) (200 * armScaleFactor);
	public static final int elbowAccel = (int) (400 * armScaleFactor);
	public static final int wristVel = (int) (400 * armScaleFactor);
	public static final int wristAccel = (int) (400 * armScaleFactor);

	public static final int armTolerance = 10; // units of encoder ticks

	// Conversion factors and quick maffs
	public static final double wheelCircumference = Math.PI * wheelDiameter;
	public static final double elevatorSprocketCircumference = Math.PI * elevatorSprocketDiameter;
	/** Multiply encoder ticks by this to convert to inches for drive train */
	public static final double encoderToInchDrive = wheelCircumference / encoderTicksPerRotation;
	/** Multiply inches by this to convert to encoder ticks for drive train */
	public static final double inchToEncoderDrive = encoderTicksPerRotation / wheelCircumference;
	/** Multiply encoder ticks by this to convert to inches for elevator */
	public static final double encoderToInchElevator = elevatorSprocketCircumference / encoderTicksPerRotation;
	/** Multiply inches by this to convert to encoder ticks for elevator */
	public static final double inchToEncoderElevator = encoderTicksPerRotation / elevatorSprocketCircumference;
	public static final double distancePerRotation = wheelBase * Math.PI / 4;

	public static final int angleTolerance = 2;
	public static final double kPStraightGyro = 0.025;

	public static final int driveTolerance = 200;
	public static final int strafeTolerance = 400;

	// Drive Train current limits
	public static final int continuousCurrentLimit = 20;
	public static final int peakCurrentLimit = 22;
	public static final int currentDuration = 50;

	// Arm current limits
	public static final int continuousCurrentLimitArm = 8;
	public static final int peakCurrentLimitArm = 10;

	/** Sensor constants */
	public static final double ultraSonic5V = 1000 * 5 / 4.8 / 25.4; // 4.88mV per 5 mm sensitivity, convert voltage to
																		// inches
	public static final double infraredScale = 1 / 0.165 / 2.54; // Divide by this to get voltage scaling factor to inches
	
	public static final int arduinoBaudRate = 9600;

	public static final double nudgeForwardDist = 6;
}
