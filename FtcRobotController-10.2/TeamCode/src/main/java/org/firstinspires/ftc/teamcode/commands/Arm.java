package org.firstinspires.ftc.teamcode.commands;


import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.CServo;
import org.firstinspires.ftc.teamcode.subsystems.Motors;
import org.firstinspires.ftc.teamcode.subsystems.myServo;

public class Arm {

    private CServo Arm;


    public Arm(CServo arm){
        Arm = arm;
    }

    public void defultPos(){


    }

    public void secLevelSpecimen(){
        Arm.set(0.5);
        //Arm.setToPos();
    }

    public void pickup(){
        Arm.set(-0.2);

    }

    public void hold()
    {

        Arm.disable();
    }
}
