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
        driveTrain = new DriveTrain(hardwareMap);  // Don't redeclare with 'DriveTrain' keyword
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
        ArrayList<Point> controlPoints = new ArrayList<>();
        controlPoints.add(new Point(0, 0, 0));
        controlPoints.add(new Point(2, 10, 0));
        controlPoints.add(new Point(4, 3, 0));
        controlPoints.add(new Point(10, 15, 0));
        controlPoints.add(new Point(11, 4, 0));
        controlPoints.add(new Point(11.1, 4.1, 0));

        // Create a cubic spline interpolator
        CubicSplineInterpolator cubicInterpolator = new CubicSplineInterpolator(controlPoints);

        // Interpolate points along the cubic spline with a resolution of 0.1
        ArrayList<Point> interpolatedPoints = cubicInterpolator.interpolate(0.2);

        // Create the motion profile generator
        MotionProfile motionProfile = new MotionProfile(1.0, 0.5, 0.02);  // Max speed 1 m/s, max accel 0.5 m/s^2

        // Generate the motion profile based on the interpolated points
        ArrayList<MotionState> profile = motionProfile.generateProfile(interpolatedPoints);

        // Print the motion profile
        for (MotionState state : profile) {
            dashboardTelemetry.addData("Interpolation", state);
            dashboardTelemetry.update();
        }



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

