package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Motors;

public class Winch {

    private Motors Winch;


    public Winch(Motors winch){
        Winch = winch;
        Winch.BreakMode();
    }

    public void home(){
        Winch.setPoint(0);
    }

    public void pickup(){
        Winch.setPoint(300);
    }

    public void LLS(){
        Winch.setPoint(2000);

    }

    public void armAngleReset(){
        Winch.encoderReset();
    }
}
