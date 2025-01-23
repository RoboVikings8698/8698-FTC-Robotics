package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Commanding;
import org.firstinspires.ftc.teamcode.commands.DriveRobot;
import org.firstinspires.ftc.teamcode.subsystems.GamePad;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.Lift;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.Motors;


//Main code
@TeleOp
public class MainTeleop extends CommandOpMode {


    //Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();  //declaration dashboard
    Telemetry dashboardTelemetry = dashboard.getTelemetry(); //declaration dashboard
    DriveTrain driveTrain;

    @Override
    public void initialize() {

        driveTrain = new DriveTrain(hardwareMap);
        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        DriveRobot DriveRobot = new DriveRobot(driveTrain, driverGamepad);

        driveTrain.setDefaultCommand(DriveRobot);

    }
}



