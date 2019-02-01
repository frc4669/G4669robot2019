/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot;

import edu.wpi.first.wpilibj.AnalogInput;

/** Class that implements analog inputs for use with ultrasonic sensor. */
public class AnalogUltrasonic extends AnalogInput {

    public static final double scaleFactor5V = 1000 * 5 / 4.8 / 25.4; // 4.88mV per 5 mm sensitivity, convert voltage to
                                                                      // inches

    public AnalogUltrasonic(int port) {
        super(port);
    }

    /**
     * @return The instantaneous distance measured by the sensor.
     */
    public double getDistance() {
        return getVoltage() * scaleFactor5V;
    }

    /** @return The instantaneous voltage value of the sensor */
    public double getVoltage() {
        return super.getVoltage();
    }
}
