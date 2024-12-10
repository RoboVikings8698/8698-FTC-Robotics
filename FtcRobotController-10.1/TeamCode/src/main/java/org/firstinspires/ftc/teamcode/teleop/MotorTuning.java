package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.Motors;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;




//This class is used to tune motors.
@TeleOp
public class MotorTuning extends OpMode {

    //motor test
    private Motors testMotor;
    private String moto_name;

    //Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();  //declaration dashboard
    Telemetry dashboardTelemetry = dashboard.getTelemetry(); //declaration dashboard


    @Override
    public void init() {


        testMotor = new Motors(hardwareMap, Constants.Motors.Winch, 1, 0, 0,0, 0, 1, 0, Constants.Motors.MotorB5202312crp,Constants.Motors.MotorB5202312rpm);
        //testMotor.CoastMode();


        //periodic to run PIC controllers (always place on the bottom)
        PeriodicScheduler.register(testMotor);
    }

    @Override
    public void loop() {
        //code that keep all periodic running
        PeriodicScheduler.runPeriodically();

        testMotor.setControlMethod(Dashboard.MotorTuning.ControlMethod);
        testMotor.tuneMotionController(Dashboard.MotorTuning.kp,0,Dashboard.MotorTuning.kd);
        testMotor.setPoint(Dashboard.MotorTuning.SetPoint);

        dashboardTelemetry.addData("motorPosition", testMotor.getPosition());
        dashboardTelemetry.addData("motorVelocity", testMotor.getVelocity());
        dashboardTelemetry.addData("setpoint", Dashboard.MotorTuning.SetPoint);
        dashboardTelemetry.update();



    }
}
