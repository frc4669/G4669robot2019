/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.misc;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * Add your docs here.
 */
public class ArduinoCommunicator {
    public SerialPort arduino;

    /** Creates the serial communication to arduino from RoboRio */
    public ArduinoCommunicator(){
        try{
            arduino = new SerialPort(Constants.arduinoBaudRate , Port.kUSB);
            System.out.println("Arduino connected on kUSB");
        } catch(Exception E){
            System.out.println("Failed to connect on kUSB, attempting kUSB1");
            try{
                arduino = new SerialPort(Constants.arduinoBaudRate , Port.kUSB1);
                System.out.println("Arduino connected on kUSB1");
            } catch(Exception E1){
                System.out.println("Failed to connect on kUSB1, attempting kUSB2");
                try{
                    arduino = new SerialPort(Constants.arduinoBaudRate , Port.kUSB2);
                    System.out.println("Arduino connected on kUSB2");
                } catch(Exception E2){
                    System.out.println("Failed to connect on kUSB2");
                }
            }
        }
    }

    public void sendString(String input){
        if(isArduinoConnected()){
            arduino.writeString(input);
            System.out.println("Sent String: " + input);
        }
    }

    public void sendRGB(int r, int g, int b){
        String input = r + "," + g + "," + b + "|";
        sendString(input);
    }

    public boolean isArduinoConnected(){
        return arduino!=null;
    }

    public String receiveString(){
        String str = arduino.readString();
        str.substring(0,str.indexOf("|"));
        return str;
    }
}
