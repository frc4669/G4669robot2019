/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot;

import org.usfirst.frc.team4669.robot.commands.auto.IncrementalClimb;
import org.usfirst.frc.team4669.robot.commands.elevator.ExtendBothElevator;
import org.usfirst.frc.team4669.robot.commands.elevator.ExtendLeftElevator;
import org.usfirst.frc.team4669.robot.commands.elevator.ExtendRightElevator;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	// Joystick variables
	private Joystick leftStick = new Joystick(RobotMap.leftJoystick);
	private Joystick rightStick = new Joystick(RobotMap.rightJoystick);
	private Joystick extremeStick = new Joystick(RobotMap.extremeJoystick);

	private Button leftButton1 = new JoystickButton(leftStick, 1), leftButton2 = new JoystickButton(leftStick, 2),
			leftButton3 = new JoystickButton(leftStick, 3), leftButton4 = new JoystickButton(leftStick, 4),
			leftButton5 = new JoystickButton(leftStick, 5), leftButton6 = new JoystickButton(leftStick, 6),
			leftButton7 = new JoystickButton(leftStick, 7), leftButton8 = new JoystickButton(leftStick, 8),
			leftButton9 = new JoystickButton(leftStick, 9), leftButton10 = new JoystickButton(leftStick, 10),
			leftButton11 = new JoystickButton(leftStick, 11);

	private Button rightButton1 = new JoystickButton(rightStick, 1), rightButton2 = new JoystickButton(rightStick, 2),
			rightButton3 = new JoystickButton(rightStick, 3), rightButton4 = new JoystickButton(rightStick, 4),
			rightButton5 = new JoystickButton(rightStick, 5), rightButton6 = new JoystickButton(rightStick, 6),
			rightButton7 = new JoystickButton(rightStick, 7), rightButton8 = new JoystickButton(rightStick, 8),
			rightButton9 = new JoystickButton(rightStick, 9), rightButton10 = new JoystickButton(rightStick, 10),
			rightButton11 = new JoystickButton(rightStick, 11);

	private Button extremeButton1 = new JoystickButton(extremeStick, 1), extremeButton2 = new JoystickButton(extremeStick, 2),
			extremeButton3 = new JoystickButton(extremeStick, 3), extremeButton4 = new JoystickButton(extremeStick, 4),
			extremeButton5 = new JoystickButton(extremeStick, 5), extremeButton6 = new JoystickButton(extremeStick, 6),
			extremeButton7 = new JoystickButton(extremeStick, 7), extremeButton8 = new JoystickButton(extremeStick, 8),
			extremeButton9 = new JoystickButton(extremeStick, 9), extremeButton10 = new JoystickButton(extremeStick, 10),
			extremeButton11 = new JoystickButton(extremeStick, 11);

	public OI() {
		if(Robot.endgameStarted){
			extremeButton8.whenPressed(new IncrementalClimb());
			extremeButton7.whenPressed(new ExtendBothElevator(0));
			extremeButton9.whenPressed(new ExtendLeftElevator(Constants.wheelElevatorOffSet));
			extremeButton10.whenPressed(new ExtendRightElevator(Constants.wheelElevatorOffSet));
			
		}
	}

	// Getting joystick values
	public double leftY() {
		double joystickValue = leftStick.getY();
		return deadzone(joystickValue, 0.09);
	}

	public double leftX() {
		double joystickValue = leftStick.getX();
		return deadzone(joystickValue, 0.09);
	}

	public double rightY() {
		double joystickValue = rightStick.getY();
		return deadzone(joystickValue, 0.09);
	}

	public double rightX() {
		double joystickValue = rightStick.getX();
		return deadzone(joystickValue, 0.09);
	}

	public double extremeX() {
		double joystickValue = extremeStick.getX();
		return deadzone(joystickValue, 0.09);
	}

	public double extremeY() {
		double joystickValue = -extremeStick.getY();
		return deadzone(joystickValue, 0.09);
	}

	public double extremeZ() {
		double joystickValue = extremeStick.getZ();
		return deadzone(joystickValue, 0.15);
	}

	public boolean getLeftRawButton(int button) {
		return leftStick.getRawButton(button);
	}

	public boolean getRightRawButton(int button) {
		return rightStick.getRawButton(button);
	}

	public boolean getExtremeRawButton(int button) {
		return extremeStick.getRawButton(button);
	}

	public Joystick getLeftStick() {
		return leftStick;
	}

	public Joystick getRightStick() {
		return rightStick;
	}

	public Joystick getExtremeStick() {
		return extremeStick;
	}

	private double deadzone(double joystickValue, double offset) {
		double joystickOffset = offset;
		double absJoystickValue = Math.abs(joystickValue);
		if (absJoystickValue > joystickOffset) {
			double speed = absJoystickValue;
			speed = (speed * speed) + joystickOffset;
			if (joystickValue > 0)
				return speed;
			else
				return -speed;
		} else {
			return 0;
		}
	}
}
