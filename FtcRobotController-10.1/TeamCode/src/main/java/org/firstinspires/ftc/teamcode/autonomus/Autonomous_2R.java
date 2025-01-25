package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;

import java.util.jar.Attributes;

@Autonomous
public class Autonomous_2R extends OpMode {
    //    DistanceSensor dsensor;
    double testDistance;
    private DriveTrain driveTrain;
    private ElapsedTime runtime = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();  //declaration dashboard
    Telemetry dashboardTelemetry = dashboard.getTelemetry(); //declaration dashboard
    private double timeComp = 0;

    @Override
    public void init()
    {
        PeriodicScheduler scheduler = new PeriodicScheduler();
        driveTrain = new DriveTrain(hardwareMap, Constants.DriveTrain.time);
        driveTrain.resetYaw();
        PeriodicScheduler.register(driveTrain);

//        dsensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

    }
//    public double distance()
//    {
//        double value = dsensor.getDistance(DistanceUnit.INCH);
//        return value;
//    }

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
        if (time >1.15){
            driveTrain.FieldOrientedDriveAuto(0,0, 0);
        } else if (time >= 0){
            driveTrain.FieldOrientedDriveAuto(0.5,270, 0);

        }
//        testDistance = dsensor.getDistance(DistanceUnit.INCH);
//        System.out.println("dsensor value: " + testDistance);


        PeriodicScheduler.runPeriodically();
    }

    public void periodic() {
        //calling pid every now on
        //posPID.calculatePID(getYaw()); //calculate pid, +90 added to compensate for joystick offset from bearing
//        posPID.calculatePID(getYaw());
//        dashboardTelemetry.addData("Distance Value", distance());
//
//
//        dashboardTelemetry.update();
    }
}