package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.comp.Check;

import org.firstinspires.ftc.teamcode.subsystems.CServo;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GamePad;
import org.firstinspires.ftc.teamcode.subsystems.Motors;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;
import org.firstinspires.ftc.teamcode.subsystems.myServo;

public class Commanding {

    private DriveTrain driveTrain;
    private Motors arm1;
    private Motors winch1;
    private Claw claw;
    private Arm arm;
    private Winch winch;
    private myServo servo;




    //initialise commands
    public Commanding(HardwareMap hardwareMap, PeriodicScheduler scheduler, com.qualcomm.robotcore.hardware.Gamepad controller1, com.qualcomm.robotcore.hardware.Gamepad controller2){

        //object initialization
        driveTrain = new DriveTrain(hardwareMap, Constants.DriveTrain.time);  // Don't redeclare with 'DriveTrain' keyword;
        servo = new myServo(hardwareMap, "claw", 0, 360);
        new GamePad(controller1,controller2); //setup controllers
        winch1 = new Motors(hardwareMap,Constants.Motors.winch, 1, -0.01, 0,0, 0, 1, 1, Constants.Motors.MotorB5202312crp, Constants.Motors.MotorB5202312rpm);
        arm1 = new Motors(hardwareMap,Constants.Motors.arm, 1, 0.03, 0,0.03, 0, 1, 1, Constants.Motors.MotorB5202312crp, Constants.Motors.MotorB5202312rpm);


        claw = new Claw(servo);
        arm = new Arm(arm1);
        winch = new Winch(winch1);


        PeriodicScheduler.register(driveTrain); //sets periodic for drivetrain
        PeriodicScheduler.register(arm1);
        PeriodicScheduler.register(winch1);
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
            arm.LLS();
            winch.pickup();
        }

        if(GamePad.c1.getA()){
            arm.pickup();
            winch.pickup();
        }


        traveling();

    }

    public void traveling(){
        driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), false, 0);
    }





}
