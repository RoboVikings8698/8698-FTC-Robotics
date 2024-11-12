package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MotionControl.CubicSplineInterpolator;
import org.firstinspires.ftc.teamcode.subsystems.MotionControl.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.MotionControl.MotionState;
import org.firstinspires.ftc.teamcode.subsystems.MotionControl.Point;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Functions;
import org.firstinspires.ftc.teamcode.subsystems.GamePad;
import org.firstinspires.ftc.teamcode.subsystems.Motors;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;

import java.util.ArrayList;


@TeleOp
public class Main1 extends OpMode {

    private DriveTrain driveTrain;
    private Motors motorLift;

    FtcDashboard dashboard = FtcDashboard.getInstance();  //declaration dashboard
    Telemetry dashboardTelemetry = dashboard.getTelemetry(); //declaration dashboard

    //when INT is hit, run once
    @Override
    public void init() {
        // Initialize class-level driveTrain and gamepadEx
        driveTrain = new DriveTrain(hardwareMap, Constants.DriveTrain.time);  // Don't redeclare with 'DriveTrain' keyword
        motorLift = new Motors(hardwareMap, Constants.Motors.MotorArm, , Constants.Motors.MA_cycleRate, Constants.Motors.MA_Kp, Constants.Motors.MA_Ki, Constants.Motors.MA_Kd, Constants.Motors.MA_KiClamp, Constants.Motors.MA_KOutClamp,  Constants.Motors.MotorB5202312crp, Constants.Motors.MotorB5202312rpm, Constants.Motors.MA_DR_StandbyMode);

        PeriodicScheduler.register(driveTrain);
        new GamePad(gamepad1,gamepad2);


    }



    //loop until start is pressed
    @Override
    public void init_loop() {
    }

    //run once, once start is pressed
    @Override
    public void start() {




    }

    //made code loop, run everything here
    @Override
    public void loop() {
        PeriodicScheduler.runPeriodically();

        driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), false, 0);

        if (GamePad.c1.getRB()){
            driveTrain.resetYaw();
        }





    }

    //Once stop is pressed run this once
    @Override
    public void stop() {
    }
}

