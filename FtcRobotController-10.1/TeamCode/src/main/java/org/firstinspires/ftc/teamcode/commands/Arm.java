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

    public void armAngleReset(){
        Arm.encoderReset();
    }

    public void pickup(){
        home();
    }

    public void scoreLLB(){
        Arm.setPoint(2100);
    }

    //this is for save mode change since we do not want to get stuck to carpet while rotating the intake mech
    public void modeChange(){
        Arm.setPoint(500);
    }

    public void scoreSpecimenLL(){
        Arm.setPoint(1000);
    }

    public void scoreSpecimenHL(){
        Arm.setPoint(2000);
    }
}
