package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.Motors;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;




//This class is used to tune motors.
@TeleOp
public class MotorTuning extends CommandOpMode {

    //motor test
    private Motors testMotor;

    //Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();  //declaration dashboard
    Telemetry dashboardTelemetry = dashboard.getTelemetry(); //declaration dashboard

    @Override
    public void initialize() {

        testMotor = new Motors(hardwareMap, "lift");



    }
}
