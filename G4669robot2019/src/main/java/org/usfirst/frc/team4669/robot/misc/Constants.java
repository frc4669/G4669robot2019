/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team4669.robot.misc;

/** Include field measurements, robot length measurements, and other numbered
 *  constants here to reduce magic numbers floating around.
 */
public class Constants {
	//Robot Constants
	public static final double wheelDiameter = 4; //in inches
	public static final double wheelBase = 22.25; //figure out real distance later
	
	public static final int encoderTicksPerRotation = 4096;
	
	
	//PID Constants
	public static final double kF = 0.3343;
	public static final double kP = 0.4;
	public static final double kI = 0.0003;
	public static final double kD = 20;
	public static final int kIZone = 50;
	
	public static final double kPGyro = 0;
	public static final double kIGyro = 0;
	public static final double kDGyro = 0.045;
	
	public static final double kFElevator = 0.977;
	public static final double kPElevator = 1.056;
	public static final double kIElevator = 0.006;
	public static final double kDElevator = 21.12;
	public static final double kIZoneElevator = 50;
	
	public static final int timeout = 10;
	public static final int baseTrajPeriodMs = 0;
	
	//Velocities & Acceleration for Motion Magic
	public static final int elevatorVel = 1100;
	public static final int elevatorAccel = 1300;
	
	public static final int elevatorDownVel = 1300;
	public static final int elevatorDownAccel = 2900;
		
	//Encoder Heights for Elevator
	public static final int elevatorSwitch = -10000;
	public static final int elevatorExchange = -2450;
	public static final int elevatorLift = -1200;
	public static final int elevatorScaleMid = -23000;
	public static final int elevatorMax = -27295;

	//Conversion factors and quick maffs
	public static final double wheelCircumference = Math.PI * wheelDiameter;
	
	public static final double encoderToInch = wheelCircumference / encoderTicksPerRotation; //Multiply encoder ticks by this to convert to inches
	public static final double inchToEncoder = encoderTicksPerRotation / wheelCircumference; //Multiply inches by this to convert to encoder ticks
	
	public static final double distancePerRotation = wheelBase*Math.PI/4;
	
	public static final int angleTolerance = 2;
	
	
}
