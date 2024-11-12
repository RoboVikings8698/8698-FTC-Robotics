package org.firstinspires.ftc.teamcode.autonomus;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GamePad;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;

@Autonomous
public class push_and_pull extends OpMode{

    private DriveTrain driveTrain;
    private double time;
    private double old_time;
    private double deltaTime;


    @Override
    public void init() {
        // Initialize class-level driveTrain and gamepadEx
        driveTrain = new DriveTrain(hardwareMap, Constants.DriveTrain.time);  // Don't redeclare with 'DriveTrain' keyword
        PeriodicScheduler.register(driveTrain);
        driveTrain.resetYaw();




    }

    @Override
    public void loop() {
        PeriodicScheduler.runPeriodically();
        time = getRuntime();


        if (time < 4) {
            driveTrain.FieldOrientedDriveAuto(0.3, 0, 45);
        }
        else if (time > 5 && time < 8){

            driveTrain.FieldOrientedDriveAuto(0.3, 180, 180);
        }
        else {
            driveTrain.FieldOrientedDriveAuto(0, 0, 0);
        }

    }
}
