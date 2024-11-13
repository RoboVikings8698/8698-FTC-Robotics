package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GamePad;
import org.firstinspires.ftc.teamcode.subsystems.Motors;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;


//Main code
@TeleOp
public class MainTeleop extends OpMode {

    //objects
    private DriveTrain driveTrain;

    //DashBoard
    FtcDashboard dashboard = FtcDashboard.getInstance();  //declaration dashboard
    Telemetry dashboardTelemetry = dashboard.getTelemetry(); //declaration dashboard



    //When Int pressed...
    @Override
    public void init() {
        // Initialize class-level driveTrain and gamepadEx
        driveTrain = new DriveTrain(hardwareMap, Constants.DriveTrain.time);  // Don't redeclare with 'DriveTrain' keyword

        new GamePad(gamepad1,gamepad2);

        //Should be placed on the bottom
        PeriodicScheduler.register(driveTrain);
    }

    //Loop until start pressed
    @Override
    public void init_loop() {
    }

    //When start pressed...
    @Override
    public void start() {




    }

    //While robot code is not stopped...
    @Override
    public void loop() {
        PeriodicScheduler.runPeriodically();

        driveTrain.FieldOrientDrive(GamePad.c1.getDriveJoy(), false, 0);

        if (GamePad.c1.getRB()){
            driveTrain.resetYaw();
        }


    }

    //Once robot stopped...
    @Override
    public void stop() {
    }
}

