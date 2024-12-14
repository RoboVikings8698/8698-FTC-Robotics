package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.CServo;
import org.firstinspires.ftc.teamcode.subsystems.myServo;


// controls robot claw
public class Claw {


    private myServo Servo;

    public Claw(myServo servo){
        Servo = servo;
    }


    public void intakeSpecimen(){
        Servo.setToPos(90);
    }

    public void releaseSpecimen(){
        Servo.setToPos(0);
    }

}
