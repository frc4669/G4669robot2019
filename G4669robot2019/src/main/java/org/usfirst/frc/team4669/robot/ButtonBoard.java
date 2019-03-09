
package org.usfirst.frc.team4669.robot;

import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.arm.ArmToPosition;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class ButtonBoard {

	private Joystick buttonBoard;
	private Button button1, button2, button4, button5, button6, button7,
	 button9, button10, button11, button12;
	public ButtonBoard() {
		buttonBoard = new Joystick(RobotMap.buttonBoard);
		button1 = new JoystickButton(buttonBoard, 1);
		button2 = new JoystickButton(buttonBoard, 2);
		
		button4 = new JoystickButton(buttonBoard, 4);
		button5 = new JoystickButton(buttonBoard, 5);
		button6 = new JoystickButton(buttonBoard, 6);
		button7 = new JoystickButton(buttonBoard, 7);
		
		button9 = new JoystickButton(buttonBoard, 9);
		button10 = new JoystickButton(buttonBoard, 10);
		button11 = new JoystickButton(buttonBoard, 11);
		button12 = new JoystickButton(buttonBoard, 12);


		button10.whenPressed(new ArmToPosition(Constants.robotToArmFront + Constants.xDistanceToPlace, Constants.hatch1Height, 90, false, false));
		button11.whenPressed(new ArmToPosition(Constants.robotToArmFront + Constants.xDistanceToPlace, Constants.hatch2Height, 90, false, false));
		button12.whenPressed(new ArmToPosition(Constants.robotToArmFront + Constants.xDistanceToPlace, Constants.hatch3Height, 90, false, false));


	}

	public boolean getButton(int buttonPort) {
		return buttonBoard.getRawButton(buttonPort);
	}

	public double getX() {
		return -buttonBoard.getRawAxis(0);
	}
	public double getY() {
		return buttonBoard.getRawAxis(1);
	}
	
	public boolean isRight() {
		return (getX() == 1);
	}
	
	public boolean isLeft() {
		return (getX() == -1);
	}
	
	public boolean isUp() {
		return (getY() == 1);
	}
	
	public boolean isDown() {
		return (getY() == -1);
	}
	
	public int getAngle() {
		if (isRight()) {
			return 0;
		}
		else if (isRight() && isUp()) {
			return 45;
		}
		else if (isUp()) {
			return 90;
		}
		else if (isLeft() && isUp()) {
			return 135;
		}
		else if (isLeft()) {
			return 180;
		}
		else if (isLeft() && isDown()) {
			return 225;
		}
		else if (isDown()) {
			return 270;
		}
		else if (isRight() && isDown()) {
			return 315;
		}
		else {
			return -1;
		}
	}
}
