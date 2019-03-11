/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot;

import edu.wpi.first.wpilibj.AnalogInput;

/** Class that implements analog inputs for use with analog distance sensors. */
public class AnalogDistanceSensor extends AnalogInput {
    double scaleFactor = 1;

    public AnalogDistanceSensor(int port) {
        super(port);
    }

    /**
     * @return The instantaneous distance measured by the sensor.
     */
    public double getDistance() {
        return getVoltage() * scaleFactor;
    }

    public void setScaleFactor(double scaleFactor){
        this.scaleFactor = scaleFactor;
    }

}
