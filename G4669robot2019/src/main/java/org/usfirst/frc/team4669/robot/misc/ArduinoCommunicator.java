/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.misc;

import java.util.Arrays;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * Add your docs here.
 */
public class ArduinoCommunicator {
    public SerialPort arduino;
    protected String buf = null;

    /** Creates the serial communication to arduino from RoboRio */
    public ArduinoCommunicator(int baudRate){
        try{
            arduino = new SerialPort(baudRate , Port.kUSB);
            System.out.println("Arduino connected on kUSB");
        } catch(Exception E){
            System.out.println("Failed to connect on kUSB, attempting kUSB1");
            try{
                arduino = new SerialPort(baudRate , Port.kUSB1);
                System.out.println("Arduino connected on kUSB1");
            } catch(Exception E1){
                System.out.println("Failed to connect on kUSB1, attempting kUSB2");
                try{
                    arduino = new SerialPort(baudRate , Port.kUSB2);
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

    public int[] receiveDistance(){
        if(arduino!=null){
            if(buf==null){
                buf = arduino.readString();
            } else{
                buf+= arduino.readString();
            }
            int i = buf.lastIndexOf("\n");
            String in = null;
            if(i!=-1){
                in = buf.substring(0,i);
                if(buf.length()>i+1)
                    buf = buf.substring(i+1);
                else
                    buf = null;
            }
            if(in != null&&in.length()>0){
                String[] str = in.split(" ");
                // System.out.println(Arrays.toString(str));
                if(str.length==3){
                    try{
                        int[] distances = {-2,-2,-2};
                        distances[0] = Integer.parseInt(str[0].trim());
                        distances[1] = Integer.parseInt(str[1].trim());
                        distances[2] = Integer.parseInt(str[2].trim());
                        return distances;
                    } catch(Exception e){
                        e.printStackTrace();
                    }
                }
            }
        }
        return null;

    }


}
