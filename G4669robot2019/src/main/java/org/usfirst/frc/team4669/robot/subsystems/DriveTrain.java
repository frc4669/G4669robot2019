/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.Vector2d;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.MecanumDriveCommand;
import org.usfirst.frc.team4669.robot.misc.Constants;
import org.usfirst.frc.team4669.robot.misc.PIDOutputWrapper;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX frontLeftMotor;
  private WPI_TalonSRX rearLeftMotor;
  private WPI_TalonSRX frontRightMotor;
  private WPI_TalonSRX rearRightMotor;

  private SpeedControllerGroup leftMotorGroup;
  private SpeedControllerGroup rightMotorGroup;
  private MecanumDrive drive;

  public PIDController gyroPID;
  private PIDOutputWrapper turnOutput;
  private Gyro analogGyro;

  int velocity = 2300; // About 200 RPM, vel units are in sensor units per 100ms
  int accel = 4600;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MecanumDriveCommand());
    // setDefaultCommand(new MySpecialCommand());
  }

  public DriveTrain() {
    frontLeftMotor = new WPI_TalonSRX(RobotMap.driveFrontLeft);
    rearLeftMotor = new WPI_TalonSRX(RobotMap.driveRearLeft);
    frontRightMotor = new WPI_TalonSRX(RobotMap.driveFrontRight);
    rearRightMotor = new WPI_TalonSRX(RobotMap.driveRearRight);

    frontLeftMotor.setSafetyEnabled(false);
    frontRightMotor.setSafetyEnabled(false);
    rearLeftMotor.setSafetyEnabled(false);
    rearRightMotor.setSafetyEnabled(false);

    analogGyro = new ADXRS450_Gyro();

    // Configuring the Gyroscope and the PID controller for it

    turnOutput = new PIDOutputWrapper();

    gyroPID = new PIDController(Constants.kPGyro, Constants.kIGyro, Constants.kDGyro, (PIDSource) analogGyro,
        turnOutput);
    gyroPID.setInputRange(0, 360);
    gyroPID.setContinuous(true);
    gyroPID.setOutputRange(-0.5, 0.5);
    gyroPID.setAbsoluteTolerance(3);

    // Configuring the motors and encoders

    frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.pidIdx, Constants.timeoutMs);
    frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.pidIdx, Constants.timeoutMs);
    rearRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.pidIdx, Constants.timeoutMs);
    rearLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.pidIdx, Constants.timeoutMs);

    // Inverts motor direction
    frontRightMotor.setInverted(false);
    frontLeftMotor.setInverted(true);
    rearRightMotor.setInverted(false);
    rearLeftMotor.setInverted(true);

    // Sets the encoder sensors to be in phase/direction with motors
    frontRightMotor.setSensorPhase(false);
    frontLeftMotor.setSensorPhase(false);
    rearRightMotor.setSensorPhase(false);
    rearLeftMotor.setSensorPhase(false);

    /* set the peak and nominal outputs, 1 means full */
    frontRightMotor.configNominalOutputForward(0, Constants.timeoutMs);
    frontRightMotor.configNominalOutputReverse(0, Constants.timeoutMs);
    frontRightMotor.configPeakOutputForward(1, Constants.timeoutMs);
    frontRightMotor.configPeakOutputReverse(-1, Constants.timeoutMs);

    frontLeftMotor.configNominalOutputForward(0, Constants.timeoutMs);
    frontLeftMotor.configNominalOutputReverse(0, Constants.timeoutMs);
    frontLeftMotor.configPeakOutputForward(1, Constants.timeoutMs);
    frontLeftMotor.configPeakOutputReverse(-1, Constants.timeoutMs);

    /* set closed loop gains in slot0 - see documentation */
    frontRightMotor.selectProfileSlot(RobotMap.slotIdx, RobotMap.pidIdx);
    frontRightMotor.config_kF(RobotMap.slotIdx, Constants.kF, Constants.timeoutMs);
    frontRightMotor.config_kP(RobotMap.slotIdx, Constants.kP, Constants.timeoutMs);
    frontRightMotor.config_kI(RobotMap.slotIdx, Constants.kI, Constants.timeoutMs);
    frontRightMotor.config_kD(RobotMap.slotIdx, Constants.kD, Constants.timeoutMs);
    frontRightMotor.config_IntegralZone(RobotMap.slotIdx, Constants.kIZone, Constants.timeoutMs);

    frontLeftMotor.selectProfileSlot(RobotMap.slotIdx, RobotMap.pidIdx);
    frontLeftMotor.config_kF(RobotMap.slotIdx, Constants.kF, Constants.timeoutMs);
    frontLeftMotor.config_kP(RobotMap.slotIdx, Constants.kP, Constants.timeoutMs);
    frontLeftMotor.config_kI(RobotMap.slotIdx, Constants.kI, Constants.timeoutMs);
    frontLeftMotor.config_kD(RobotMap.slotIdx, Constants.kD, Constants.timeoutMs);
    frontLeftMotor.config_IntegralZone(RobotMap.slotIdx, Constants.kIZone, Constants.timeoutMs);

    /* set acceleration and vcruise velocity - see documentation */
    frontRightMotor.configMotionCruiseVelocity(velocity, Constants.timeoutMs);
    frontRightMotor.configMotionAcceleration(accel, Constants.timeoutMs);
    frontRightMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeoutMs);

    frontLeftMotor.configMotionCruiseVelocity(velocity, Constants.timeoutMs);
    frontLeftMotor.configMotionAcceleration(accel, Constants.timeoutMs);
    frontLeftMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeoutMs);

    // Set Current limit
    frontRightMotor.configContinuousCurrentLimit(20, Constants.timeoutMs);
    frontRightMotor.configPeakCurrentLimit(22, Constants.timeoutMs);
    frontRightMotor.configPeakCurrentDuration(50, Constants.timeoutMs);
    frontRightMotor.enableCurrentLimit(true);

    frontLeftMotor.configContinuousCurrentLimit(20, Constants.timeoutMs);
    frontLeftMotor.configPeakCurrentLimit(22, Constants.timeoutMs);
    frontLeftMotor.configPeakCurrentDuration(50, Constants.timeoutMs);
    frontLeftMotor.enableCurrentLimit(true);

    rearRightMotor.configContinuousCurrentLimit(20, Constants.timeoutMs);
    rearRightMotor.configPeakCurrentLimit(22, Constants.timeoutMs);
    rearRightMotor.configPeakCurrentDuration(50, Constants.timeoutMs);
    rearRightMotor.enableCurrentLimit(true);

    rearLeftMotor.configContinuousCurrentLimit(20, Constants.timeoutMs);
    rearLeftMotor.configPeakCurrentLimit(22, Constants.timeoutMs);
    rearLeftMotor.configPeakCurrentDuration(50, Constants.timeoutMs);
    rearLeftMotor.enableCurrentLimit(true);

    drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    drive.setRightSideInverted(false);
    drive.setSafetyEnabled(false);
  }

  /**
   * Cartesian drive method that specifies speeds in terms of the field
   * longitudinal and lateral directions, using the drive's angle sensor to
   * automatically determine the robot's orientation relative to the field.
   * <p>
   * Using this method, the robot will move away from the drivers when the
   * joystick is pushed forwards, and towards the drivers when it is pulled
   * towards them - regardless of what direction the robot is facing.
   * 
   * @param ySpeed       The speed that the robot should drive left and right.
   *                     [-1.0..1.0]
   * @param xSpeed       The speed that the robot should drive fowards and
   *                     backwards. [-1.0..1.0]
   * @param rotationRate The rate of rotation for the robot that is completely
   *                     independent of the translation. [-1.0..1.0]
   * @param gyroAngle    The angle of the robot relative to the field in degrees
   */
  public void fieldOrientedDrive(double ySpeed, double xSpeed, double rotationRate, double gyroAngle) {
    drive.driveCartesian(ySpeed, xSpeed, rotationRate, gyroAngle);
  }

  /**
   * Cartesian drive method that specifies speeds in terms of the field
   * longitudinal and lateral directions, using the drive's angle sensor to
   * automatically determine the robot's orientation relative to the field.
   * <p>
   * Using this method, the robot will move away from the drivers when the
   * joystick is pushed forwards, and towards the drivers when it is pulled
   * towards them - regardless of what direction the robot is facing.
   * 
   * @param ySpeed       The speed that the robot should drive left and right.
   *                     [-1.0..1.0]
   * @param xSpeed       The speed that the robot should drive fowards and
   *                     backwards. [-1.0..1.0]
   * @param rotationRate The rate of rotation for the robot that is completely
   *                     independent of the translation. [-1.0..1.0]
   */
  public void robotOrientedDrive(double ySpeed, double xSpeed, double rotation) {
    drive.driveCartesian(ySpeed, xSpeed, rotation);
  }

  /**
   * Polar drive method that specifies speeds in terms of magnitude and direction.
   * This method does not use the drive's angle sensor.
   * 
   * @param magnitude    The speed that the robot should drive in a given
   *                     direction.
   * @param direction    The direction the robot should drive in degrees. The
   *                     direction and magnitude are independent of the rotation
   *                     rate.
   * @param rotationRate The rate of rotation for the robot that is completely
   *                     independent of the magnitude or direction. [-1.0..1.0]
   */
  public void polarDrive(double magnitude, double direction, double rotationRate) {
    drive.drivePolar(magnitude, direction, rotationRate);
  }

  public void driveRearLeft(double percentage) {
    rearLeftMotor.set(percentage);
  }

  public void driveRearRight(double percentage) {
    rearRightMotor.set(percentage);
  }

  public void driveFrontLeft(double percentage) {
    frontLeftMotor.set(percentage);
  }

  public void driveFrontRight(double percentage) {
    frontRightMotor.set(percentage);
  }

  public void calibrateGyro() {
    analogGyro.calibrate();
  }

  public void resetGyro() {
    analogGyro.reset();
  }

  public int getFrontLeftEncoder() {
    return frontLeftMotor.getSensorCollection().getQuadraturePosition();
  }

  public int getFrontRightEncoder() {
    return frontRightMotor.getSensorCollection().getQuadraturePosition();
  }

  public int getRearLeftEncoder() {
    return rearLeftMotor.getSensorCollection().getQuadraturePosition();
  }

  public int getRearRightEncoder() {
    return rearRightMotor.getSensorCollection().getQuadraturePosition();
  }

  public double getFrontLeftEncoderSpeed() {
    return frontLeftMotor.getSensorCollection().getQuadratureVelocity();
  }

  public double getFrontRightEncoderSpeed() {
    return frontRightMotor.getSensorCollection().getQuadratureVelocity();
  }

  public double getRearLeftEncoderSpeed() {
    return rearLeftMotor.getSensorCollection().getQuadratureVelocity();
  }

  public double getRearRightEncoderSpeed() {
    return rearRightMotor.getSensorCollection().getQuadratureVelocity();
  }

  public void zeroEncoders() {
    frontLeftMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeoutMs);
    rearLeftMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeoutMs);
    frontRightMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeoutMs);
    rearRightMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeoutMs);
  }

  public void stop() {
    drive.driveCartesian(0, 0, 0);
  }

  public void enableTurnPID() {
    gyroPID.reset();
    gyroPID.enable();
  }

  public void disableTurnPID() {
    gyroPID.disable();
  }

  public void setTurnAngle(double angle) {
    gyroPID.setSetpoint(angle);
  }

  public double getAngle() {
    return analogGyro.getAngle();
  }

  // public void fieldDrive(double ySpeed, double xSpeed, double zRotation, double
  // gyroAngle) {
  // // Compensate for gyro angle.
  // Vector2d input = new Vector2d(ySpeed, xSpeed);
  // input.rotate(-gyroAngle);

  // double m_maxOutput = 1;

  // double[] wheelSpeeds = new double[4];

  // wheelSpeeds[0] = input.x + input.y + zRotation; // frontLeft
  // wheelSpeeds[1] = -input.x + input.y - zRotation; // frontRight
  // wheelSpeeds[2] = -input.x + input.y + zRotation; // rearLeft
  // wheelSpeeds[3] = input.x + input.y - zRotation; // rearRight

  // normalize(wheelSpeeds);

  // frontLeftMotor.set(wheelSpeeds[0] * m_maxOutput);
  // frontRightMotor.set(wheelSpeeds[1] * m_maxOutput);
  // rearLeftMotor.set(wheelSpeeds[2] * m_maxOutput);
  // rearRightMotor.set(wheelSpeeds[3] * m_maxOutput);
  // }

  // public void robotDrive(double ySpeed, double xSpeed, double zRotation) {
  // fieldDrive(ySpeed, xSpeed, zRotation, 0);
  // }

  private void normalize(double[] wheelSpeeds) {
    double maxMagnitude = Math.abs(wheelSpeeds[0]);
    for (int i = 1; i < wheelSpeeds.length; i++) {
      double temp = Math.abs(wheelSpeeds[i]);
      if (maxMagnitude < temp) {
        maxMagnitude = temp;
      }
    }
    if (maxMagnitude > 1.0) {
      for (int i = 0; i < wheelSpeeds.length; i++) {
        wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
      }
    }
  }

  // public double getAngleBound180() {
  // double angle = analogGyro.getAngle();
  // while (angle > 180) {
  // angle -= 180;
  // }
  // while (angle < -180) {
  // angle += 180;
  // }
  // return angle;
  // }

  // public double getAngleBound360() {
  // double angle = analogGyro.getAngle();
  // while (angle > 360) {
  // angle -= 360;
  // }
  // while (angle < 0) {
  // angle += 360;
  // }
  // return angle;
  // }

  public boolean getTurnDone() {
    return gyroPID.onTarget();
  }

  public double getTurnPIDError() {
    return gyroPID.getError();
  }

  public double getTurnOutput() {
    return turnOutput.getOutput();
  }

  public void feedWatchdog() {
    drive.feedWatchdog();
  }

  public void driveStraightGyro(double power, double targetedAngle, double kP) {
    double error = getAngle() - targetedAngle;
    double turnPower = error * kP;
    robotOrientedDrive(0, power, turnPower);

  }
}
