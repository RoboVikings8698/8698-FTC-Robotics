package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.CServo;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GamePad;
import org.firstinspires.ftc.teamcode.subsystems.Motors;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;
import org.firstinspires.ftc.teamcode.subsystems.myServo;

public class Commanding {

    private DriveTrain driveTrain;
    private CServo servo1;
    private myServo servo2;

    //commands
    private Claw claw;
    private Arm arm;

    //initialise commands
    public Commanding(HardwareMap hardwareMap, PeriodicScheduler scheduler, com.qualcomm.robotcore.hardware.Gamepad controller1, com.qualcomm.robotcore.hardware.Gamepad controller2){

        //object initialization
        driveTrain = new DriveTrain(hardwareMap, Constants.DriveTrain.time);  // Don't redeclare with 'DriveTrain' keyword
        servo1 = new CServo(hardwareMap, Constants.Servos.cServo1);
        servo2 = new myServo(hardwareMap,Constants.Servos.Servo1, 0, 360);

        new GamePad(controller1,controller2); //setup controllers

        //command initialization
        claw = new Claw(servo2);
        arm = new Arm(servo1);


        //runs drive train periodically
        PeriodicScheduler.register(driveTrain); //sets periodic for drivetrain
    }

    public void CommandingRun(){
        CheckUserInput(); //checks for user control inputs
    }


    public void CheckUserInput(){
        //reset gyro to zero
        if (GamePad.c1.getLB()){
            driveTrain.resetYaw();
        }

        //intake
        if (GamePad.c1.getRB()) {
            arm.pickup();
            claw.intakeSpecimen();
        }else{
            claw.releaseSpecimen();
        }


        if(GamePad.c1.getX()){
            arm.secLevelSpecimen();
            driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), true, 0);
        }
        else{
            driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), false, 0);

        }
    }

}
