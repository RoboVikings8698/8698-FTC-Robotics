package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class myServo {

    private ServoEx servo;

    //constructor
    public myServo(HardwareMap hardwareMap, String sName, double minA, double maxA){
        servo = new SimpleServo(hardwareMap, sName, minA, maxA);
    }

    //functions set motor servo motor power like dc motor
    public void setToPos(double pos){
        servo.turnToAngle(pos);
    }


    public void byAngle(double angle){
        servo.rotateByAngle(angle);

    }
}
