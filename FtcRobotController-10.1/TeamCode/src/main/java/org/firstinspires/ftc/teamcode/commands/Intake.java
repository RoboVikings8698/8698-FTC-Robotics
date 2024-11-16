package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.CServo;
import org.firstinspires.ftc.teamcode.subsystems.myServo;

public class Intake {

    private myServo Servo;
    private CServo cServo;

    public Intake(myServo servo, CServo cservo){
        Servo = servo;
        cServo = cservo;
    }


    public void intakeSpecimen(){
        cServo.set(-1);
    }

    public void releaseSpecimen(){
        cServo.set(1);
    }

    public void roller_hold(){
        //cServo.set(-0.1);
        releaseSpecimen();
    }

    public void setToIntake(){
        Servo.setToPos(135);
    }

    public void setToHome(){
        Servo.setToPos(0);
    }

    public void setToSpecimenScore(){
        Servo.setToPos(5);
    }

    public void setToScoreLLB(){
        setToIntake();
    }
}
