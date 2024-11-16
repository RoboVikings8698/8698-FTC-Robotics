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
    private CServo intake;
    private myServo yawIntake;
    private Motors arm;


    //commands
    private Intake Intake;
    private Arm Arm;

    //initialise commands
    public Commanding(HardwareMap hardwareMap, PeriodicScheduler scheduler, com.qualcomm.robotcore.hardware.Gamepad controller1, com.qualcomm.robotcore.hardware.Gamepad controller2){

        //object initialization
        driveTrain = new DriveTrain(hardwareMap, Constants.DriveTrain.time);  // Don't redeclare with 'DriveTrain' keyword
        intake = new CServo(hardwareMap, Constants.Servos.cServo1);
        yawIntake = new myServo(hardwareMap,Constants.Servos.Servo1, 0, 360);
        new GamePad(controller1,controller2); //setup controllers
        arm = new Motors(hardwareMap, Constants.Motors.Arm, Constants.Motors.Arm_pidCycle, Constants.Motors.kp, 0, Constants.Motors.kd, 0, 1, Constants.Motors.Arm_pidMode, Constants.Motors.MotorB5202312crp, Constants.Motors.MotorB5202312rpm);

        //command initialization
        Intake = new Intake(yawIntake,intake);
        Arm = new Arm(arm);


        PeriodicScheduler.register(driveTrain); //sets periodic for drivetrain
        PeriodicScheduler.register(arm); //sets periodic for the arm
    }


    public void CommandingRun(){



    }

    //home everything
    public void Home(){
        Arm.modeChange();
        Intake.setToHome();
        Arm.home();
    }


    public void CheckUserInput(){
        //reset gyro to zero
        if (GamePad.c1.getLB()){
            driveTrain.resetYaw();
        }

        //roll in the intake
        if (GamePad.c1.getRB()){
            Intake.intakeSpecimen();
        } else if(GamePad.c1.getB()){
            Intake.releaseSpecimen();
        }
        else{
            Intake.roller_hold();
        }


        //intake mode
        if (GamePad.c1.getA()){
            Intake.setToIntake();
            Arm.pickup();
            driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), false, 0);

        }
        //score high box
        else if(GamePad.c1.getY()){
            Arm.scoreLLB();
            Intake.setToScoreLLB();
            driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), true, 135);
        }
        else if(GamePad.c1.getX()){
            Arm.scoreSpecimenLL();
            Intake.setToSpecimenScore();
        }
        else {
            traveling();
        }




    }

    public void traveling(){
        Arm.modeChange();
        Intake.setToSpecimenScore();
        driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), false, 0);
    }





}
