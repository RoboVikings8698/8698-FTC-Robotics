package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Functions;
import org.firstinspires.ftc.teamcode.subsystems.PeriodicScheduler;




@TeleOp
public class Main1 extends OpMode {

    private DriveTrain driveTrain;
    private GamepadEx gamepadEx;


    //when INT is hit, run once
    @Override
    public void init() {
        // Initialize class-level driveTrain and gamepadEx
        driveTrain = new DriveTrain(hardwareMap);  // Don't redeclare with 'DriveTrain' keyword
        PeriodicScheduler.register(driveTrain);
        gamepadEx = new GamepadEx(gamepad1);  // Initialize class-level gamepadEx
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

        //controller input management
        double LSY = -Functions.DeadZone(gamepadEx.getLeftY(), Constants.Controllers.controllerDeadZone);
        double LSX = Functions.DeadZone(gamepadEx.getLeftX(), Constants.Controllers.controllerDeadZone); //gets each controller's inputs
        double RSY = -Functions.Exponential(Functions.DeadZone(gamepadEx.getRightY(), Constants.Controllers.controllerDeadZone));
        double RSX = Functions.Exponential(Functions.DeadZone(gamepadEx.getRightX(), Constants.Controllers.controllerDeadZone));

        //driveTrain.directDrive(LSY, LSX, RSY, RSX);
        driveTrain.FieldOrientDrive(LSY, LSX, RSY, RSX);

        if (gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER)){ driveTrain.resetYaw();}

    }

    //Once stop is pressed run this once
    @Override
    public void stop() {
    }
}

