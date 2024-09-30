package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.PeriodicScheduler;

@TeleOp
public class Main1 extends OpMode {

    private DriveTrain driveTrain;


    static volatile double runtimeAve = 0;

    //when INT is hit, run once
    @Override
    public void init() {
        //proper initialization of DriveTrain
        driveTrain = new DriveTrain(hardwareMap);  // Create an instance of DriveTrains
        PeriodicScheduler.register(driveTrain);



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



        //DEV TOOLS
        driveTrain.DirectMotorControl(Dashboard.DriveTrain.motorNum, Dashboard.DriveTrain.motor_RawPower, Dashboard.DriveTrain.Kp, Dashboard.DriveTrain.Ki, Dashboard.DriveTrain.Kd, Dashboard.DriveTrain.KiClamp, Dashboard.DriveTrain.KOutClamp, Dashboard.DriveTrain.KOutRateClamp, Dashboard.DriveTrain.SetPoint);

    }

    //Once stop is pressed run this once
    @Override
    public void stop() {
    }
}

