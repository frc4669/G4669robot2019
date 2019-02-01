package org.usfirst.frc.team4669.robot.misc;

import org.usfirst.frc.team4669.robot.Robot;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class VisionEntries {
    private int x = 0, y = 0, width = 0, height = 0;

    public VisionEntries() {
        updateEntries();
    }

    public void updateEntries() {
        if (Robot.visionTable.containsKey("orangeX"))
            x = Robot.visionTable.getEntry("orangeX").getNumber(0).intValue();
        if (Robot.visionTable.containsKey("orangeY"))
            y = Robot.visionTable.getEntry("orangeY").getNumber(0).intValue();
        if (Robot.visionTable.containsKey("orangeWidth"))
            width = Robot.visionTable.getEntry("orangeWidth").getNumber(0).intValue();
        if (Robot.visionTable.containsKey("orangeHeight"))
            height = Robot.visionTable.getEntry("orangeHeight").getNumber(0).intValue();
    }

    public int getX() {
        updateEntries();
        return x;
    }

    public int getY() {
        updateEntries();
        return y;
    }

    public int getWidth() {
        updateEntries();
        return width;
    }

    public int getHeight() {
        updateEntries();
        return height;
    }

    public double getDistance() {
        updateEntries();
        return width;
    }

    public boolean isObjectDetected() {
        return !(width ==  0 ||height == 0);
    }
}
