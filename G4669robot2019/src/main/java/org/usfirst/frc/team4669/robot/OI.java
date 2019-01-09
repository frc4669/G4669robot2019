/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot;

import edu.wpi.first.wpilibj.Joystick;

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
	
	//Joystick variables
		private Joystick leftStick;
		private Joystick rightStick;

		public OI() {
			//Mapping joysticks
			leftStick = new Joystick(RobotMap.leftJoystick);
			rightStick = new Joystick(RobotMap.rightJoystick);

		}

		//Getting joystick values
		public double leftY() {
			double joystickValue = leftStick.getY();
			return deadzone(joystickValue);
		}
		public double leftX() {
			double joystickValue = leftStick.getX();
			return deadzone(joystickValue);
		}
		public double rightY() {
			double joystickValue = rightStick.getY();
			return deadzone(joystickValue);
		}
		public double rightX() {
			double joystickValue = rightStick.getX();
			return deadzone(joystickValue);
		}
		
		public boolean getLeftRawButton(int button) {
			return leftStick.getRawButton(button);
		}
		public boolean getRightRawButton(int button) {
			return rightStick.getRawButton(button);
		}
		
		public Joystick getLeftStick() {
			return leftStick;
		}
		public Joystick getRightStick() {
			return rightStick;
		}
		
		private double deadzone(double joystickValue) {
			double joystickOffset = 0.075;
			double absJoystickValue = Math.abs(joystickValue);
			if (absJoystickValue > joystickOffset) {
				double speed = absJoystickValue;
				speed = (speed*speed) + joystickOffset;
				if (joystickValue > 0) 
					return speed;
				else
					return -speed;
			}
			else {
				return 0;
			}
		}
}
