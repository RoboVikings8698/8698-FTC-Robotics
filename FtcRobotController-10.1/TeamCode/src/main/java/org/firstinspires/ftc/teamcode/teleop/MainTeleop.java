package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Commanding;
import org.firstinspires.ftc.teamcode.commands.DriveRobot;
import org.firstinspires.ftc.teamcode.subsystems.GamePad;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;
import org.firstinspires.ftc.teamcode.subsystems.myServo;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.Lift;
import org.firstinspires.ftc.teamcode.teleop.Subsystem.Motors;


@TeleOp
public class MainTeleop extends CommandOpMode {

    // Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DriveTrain driveTrain;
    Lift lift;

    @Override
    public void initialize() {

        // Initialize subsystems
        driveTrain = new DriveTrain(hardwareMap);
        lift = new Lift(hardwareMap);

        // Initialize the gamepad and set up the commands
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        DriveRobot driveRobot = new DriveRobot(driveTrain, driverGamepad);

        myServo servo = new myServo(hardwareMap, "servo", 0, 90);

        // Create the pickup command
        SequentialCommandGroup pickup = new SequentialCommandGroup(
                new InstantCommand(() -> servo.setToPos(0)),
                new InstantCommand(() -> lift.pickup()),
                new InstantCommand(() -> servo.setToPos(90)),
                new WaitCommand(1000),
                new InstantCommand(() -> lift.liftHome())
        );

        // Create the score command
        SequentialCommandGroup score = new SequentialCommandGroup(
                new InstantCommand(() -> lift.scoreLowBucket()),
                new WaitCommand(1000),
                new InstantCommand(() -> lift.scoreLowBucket())
        );

         InstantCommand resetGyro = new InstantCommand(() ->
                 driveTrain.resetYaw());


        // Button mappings
        Button aButton = driverGamepad.getGamepadButton(GamepadKeys.Button.A);
        Button yButton = driverGamepad.getGamepadButton(GamepadKeys.Button.Y);
        Button rbButton = driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);

        // When the buttons are pressed, schedule the respective commands
        aButton.whenPressed(() -> {
            schedule(pickup); // This ensures the command gets scheduled when button is pressed
        });

        rbButton.whenPressed(() -> {
            schedule(resetGyro); // This ensures the command gets scheduled when button is pressed
        });

        yButton.whenPressed(() -> {
            schedule(score); // This schedules the score command when Y button is pressed
        });



        // You can also ensure that the default command (drive robot) is scheduled continuously
        // during the TeleOp loop
        driveTrain.setDefaultCommand(driveRobot);
    }

}



