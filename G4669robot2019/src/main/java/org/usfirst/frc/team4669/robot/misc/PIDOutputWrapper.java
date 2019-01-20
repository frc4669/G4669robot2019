package org.usfirst.frc.team4669.robot.misc;

import edu.wpi.first.wpilibj.PIDOutput;

public class PIDOutputWrapper implements PIDOutput {

	private double output;

	public void pidWrite(double output) {
		this.output = output;
	}

	public double getOutput() {
		return output;
	}

}
