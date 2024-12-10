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
    private Motors lift;
    private Motors winch;



    //initialise commands
    public Commanding(HardwareMap hardwareMap, PeriodicScheduler scheduler, com.qualcomm.robotcore.hardware.Gamepad controller1, com.qualcomm.robotcore.hardware.Gamepad controller2){

        //object initialization
        driveTrain = new DriveTrain(hardwareMap, Constants.DriveTrain.time);  // Don't redeclare with 'DriveTrain' keyword;
        new GamePad(controller1,controller2); //setup controllers
        lift = new Motors(hardwareMap,Constants.Motors.Lift,Constants.Motors.MotorB5202312crp, Constants.Motors.MotorB5202312rpm);
        winch = new Motors(hardwareMap,Constants.Motors.Winch,Constants.Motors.MotorB5202312crp, Constants.Motors.MotorB5202312rpm);
        lift.BreakMode();
        winch.BreakMode();

        PeriodicScheduler.register(driveTrain); //sets periodic for drivetrain
    }


    public void CommandingRun(){
        CheckUserInput();

    }




    public void CheckUserInput(){
        //reset gyro to zero
        if (GamePad.c1.getLB()){
            driveTrain.resetYaw();
        } else if(GamePad.c1.getY()){
            traveling();
            lift.set(1);

        } else if(GamePad.c1.getA()) {
            traveling();
            lift.set(-1);

        } else if(GamePad.c1.getB()) {
            traveling();
            winch.set(-1);

        } else if(GamePad.c1.getX()) {
            traveling();
            winch.set(1);

        }else{
            traveling();
            winch.set(0);
            lift.set(0);
        }

    }

    public void traveling(){
        driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), false, 0);
    }





}
