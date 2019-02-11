/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team4669.robot.misc;

/**
 * Include field measurements, robot length measurements, and other numbered
 * constants here to reduce magic numbers floating around.
 */
public class Constants {
	// Robot Constants
	public static final double wheelDiameter = 4; // in inches
	public static final double elevatorSprocketDiameter = 1.775; // in inches
	public static final double wheelBase = 22.25; // figure out real distance later
	public static final int encoderTicksPerRotation = 4096;
	public static final double shoulderLength = 29;
	public static final double elbowLength = 21;
	public static final double wristLength = 0;
	public static final double shoulderGearRatio = 3;
	public static final double elbowGearRatio = 3;
	public static final double wristGearRatio = 2;

	public static final int pixyWidth = 320;
	public static final int pixyHeight = 200;
	public static final double pixyFocalLength = 2.8 / 25.4; // 2.8mm to inches
	public static final double pixyFOV = 75; // 75 degrees

	public static final int ballDiameter = 13;

	// Position Constants, figure out actual numbers later
	public static final double elevatorTolerance = 100; // Units in encoder position
	public static final double level2HeightInches = 6; // Rest of the units are inches
	public static final double wheelElevatorOffSet = 0.5;
	public static final double climbDriveDistance = 3;
	public static final double climbDriveDistance2 = 15;
	public static final double climbDriveDistance3 = 5;
	public static final double level3HeightInches = 19;

	public static final double armGrabBallX = 15;
	public static final double armGrabBallY = 6.5;

	public static final double startingShoulderAngle = 70;// Units in degrees, not legitimate numbers
	public static final double startingElbowAngle = -95;
	public static final double startingWristAngle = 0;

	// Constants for Pathfinder
	public static final double maxVel = 77.4; // units in inches

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
	public static final double[] elevatorPID = { 0.977, 1.056, 0.006, 21.12, 50 };

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
	public static final double[] elevatorWheelPID = { 1, 0, 0, 0, 50 };

	public static final int timeout = 10;
	public static final int baseTrajPeriodMs = 0;

	// Velocities & Acceleration for Motion Magic
	public static final int elevatorVel = 1100;
	public static final int elevatorAccel = 1300;

	public static final int elevatorDownVel = 1300;
	public static final int elevatorDownAccel = 2900;

	public static final int elevatorWheelVel = 0;
	public static final int elevatorWheelAccel = 0;

	public static final int driveVel = 2300;
	public static final int driveAccel = 4600;

	public static final double armScaleFactor = 1;

	public static final int shoulderVel = (int) (150 * armScaleFactor);
	public static final int shoulderAccel = (int) (300 * armScaleFactor);
	public static final int elbowVel = (int) (200 * armScaleFactor);
	public static final int elbowAccel = (int) (400 * armScaleFactor);
	public static final int wristVel = (int) (500 * armScaleFactor);
	public static final int wristAccel = (int) (1500 * armScaleFactor);

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

	// Drive Train current limits
	public static final int continuousCurrentLimit = 20;
	public static final int peakCurrentLimit = 22;
	public static final int currentDuration = 50;

	// Arm current limits
	public static final int continuousCurrentLimitArm = 8;
	public static final int peakCurrentLimitArm = 10;

}
