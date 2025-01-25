package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GamePad;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.Motors;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;
import org.firstinspires.ftc.teamcode.subsystems.myServo;

public class Commanding {

    private DriveTrain driveTrain;
    private Motors arm1;
    private Motors winch1;
    private Claw claw;
    private myServo servo;
    private int counter;




    //initialise commands
    public Commanding(HardwareMap hardwareMap, PeriodicScheduler scheduler, com.qualcomm.robotcore.hardware.Gamepad controller1, com.qualcomm.robotcore.hardware.Gamepad controller2){

        //object initialization
        driveTrain = new DriveTrain(hardwareMap);  // Don't redeclare with 'DriveTrain' keyword;
        //servo = new myServo(hardwareMap, "claw", 0, 360);
        new GamePad(controller1,controller2); //setup controllers
        //winch1 = new Motors(hardwareMap,Constants.Motors.winch, 1, -0.01, 0,0, 0, 1, 1, Constants.Motors.MotorB5202312crp, Constants.Motors.MotorB5202312rpm);
        //arm1 = new Motors(hardwareMap,Constants.Motors.arm, 1, 0.03, 0,0.03, 0, 1, 1, Constants.Motors.MotorB5202312crp, Constants.Motors.MotorB5202312rpm);


        claw = new Claw(servo);
        //arm = new Arm(arm1);
        //winch = new Winch(winch1);


        //PeriodicScheduler.register(driveTrain); //sets periodic for drivetrain
        //PeriodicScheduler.register(arm1);
        //PeriodicScheduler.register(winch1);
    }


    public void CommandingRun(){
        CheckUserInput();

    }




    public void CheckUserInput(){


        if (GamePad.c1.getLB()){
            driveTrain.resetYaw();
        }

        if (GamePad.c1.getRB()){
            claw.intakeSpecimen();
        }else{
            claw.releaseSpecimen();
        }

        if(GamePad.c1.getY())
        {
        }

        if(GamePad.c1.getA()){
            //arm.pickup();
            //winch.pickup();
            counter = counter+1;
        }

        switch (counter){
            case 1:
                //winch.goToLow();
                break;
            case 2:
                //winch.goToHigh();
                break;
            case 3:
                //winch.home();
                break;

        }
        /*
        if(counter == 1){
            winch.goToLow();
        }

        else if(counter == 2){
            winch.goToHigh();
        }

        else if(counter == 3){
            winch.home();
        }*/




        traveling();

    }

    public void traveling(){
        driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), false, 0);
    }





}
