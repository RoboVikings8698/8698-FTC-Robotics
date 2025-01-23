package org.firstinspires.ftc.teamcode.teleop.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Constants;

public class Lift extends SubsystemBase {
    Motors lift; //create motor lift

    public Lift(HardwareMap hardwaremap){
        //lift = new Motors(hardwaremap,Constants.Motors.lift, 1, 0, 0,0, 0, 1, 0, Constants.Motors.MotorB5202312crp,Constants.Motors.MotorB5202312rpm);

    }

    public void liftHome(){
        //lift.setPoint(0);
    }

    public void firstLevel(){
        //lift.setPoint(0);
    }

    public void secondLevel(){
        //lift.setPoint(0);
    }

    public void thirdLevel(){
        //lift.setPoint(0);
    }



    @Override
    public void periodic(){

    }
}
