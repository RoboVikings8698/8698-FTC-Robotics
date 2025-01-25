package org.firstinspires.ftc.teamcode.teleop.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Constants;

public class Lift extends SubsystemBase {
    Motors lift; //create motor lift

    public Lift(HardwareMap hardwaremap){
        lift = new Motors(hardwaremap,Constants.Motors.lift);
        lift.pidEnable();
        lift.pidTune(-0.05,0,0,0.0001);
        lift.BreakMode();
    }

    public void liftHome(){
        lift.set(100);
    }

    public void pickup(){
        lift.set(0);
    }

    public void scoreLowBucket(){
        lift.set(2000);
    }


    @Override
    public void periodic(){

    }
}
