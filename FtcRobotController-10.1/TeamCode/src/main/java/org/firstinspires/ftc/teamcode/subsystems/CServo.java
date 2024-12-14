package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CServo {

    private CRServo servo;

    public CServo(HardwareMap hardwareMap, String sName){
        servo = new CRServo(hardwareMap,sName);
    }

    public void set(double output){
        servo.set(output);
    }


    public void disable(){
        servo.disable();
    }
}
