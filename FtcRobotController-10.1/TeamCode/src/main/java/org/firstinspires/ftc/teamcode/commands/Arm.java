package org.firstinspires.ftc.teamcode.commands;


import org.firstinspires.ftc.teamcode.subsystems.Motors;

public class Arm {

    private Motors Arm;


    public Arm(Motors arm){
        Arm = arm;
        Arm.BreakMode();
    }

    public void home(){
        Arm.setPoint(0);
    }

    public void pickup(){
        Arm.setPoint(110);
    }

    public void LLS(){
        Arm.setPoint(20);

    }

    public void armAngleReset(){
        Arm.encoderReset();
    }
}
