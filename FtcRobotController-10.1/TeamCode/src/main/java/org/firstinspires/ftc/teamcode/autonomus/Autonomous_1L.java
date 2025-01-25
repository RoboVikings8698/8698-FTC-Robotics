package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Constants;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.DriveTrain;

@Autonomous
public class Autonomous_1L extends OpMode {

    private DriveTrain driveTrain;
    private ElapsedTime     runtime = new ElapsedTime();

    private double timeComp = 0;

    @Override
    public void init() {

        driveTrain = new DriveTrain(hardwareMap);
        driveTrain.resetYaw();

    }

    //When start pressed...
    @Override
    public void start() {

        try {
            Thread.sleep(0);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        timeComp = runtime.seconds();

    }


    @Override
    public void loop() {
        //double time = resetRuntime();
        double time = runtime.seconds()-timeComp;

        //Comp-bot uses 0 for YAW and Test-bot uses 270 for YAW
        //2.5s could be for if someone is to the right of us also parking
        if (time >3.6) {
            driveTrain.FieldOrientedDriveAuto(0,0, 0);
        } else if (time >= 0){
            driveTrain.FieldOrientedDriveAuto(0.5,270, 0);

        }


    }
}
