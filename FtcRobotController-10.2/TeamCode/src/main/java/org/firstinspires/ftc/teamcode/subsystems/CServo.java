package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

//This is class is created to manage continues servos.
//just run constructor and set -1 to 1 to control direction and speed of the servo motor, like standard dc motor

public class CServo {

    private CRServo servo;

    //constructor
    public CServo(HardwareMap hardwareMap, String sName){
        servo = new CRServo(hardwareMap,sName);
    }

    //set power
    public void set(double output){
        servo.set(output);
    }

    //disable power to motor
    public void disable(){
        servo.disable();
    }
}
