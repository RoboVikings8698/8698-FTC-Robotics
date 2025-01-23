package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.teleop.Subsystem.DriveTrain;

import java.util.ArrayList;
import java.util.concurrent.Executor;

public class DriveRobot extends CommandBase {
    DriveTrain driveTrain;
    GamepadEx gamepad;

        public DriveRobot(DriveTrain driveTrain, GamepadEx gamepad){
            this.driveTrain = driveTrain;
            this.gamepad = gamepad;

            addRequirements(driveTrain);
        }

        @Override
        public void execute(){
            double joystickInput = gamepad.getLeftX();
            ArrayList<Double> DRcontrol = new ArrayList<>();
            DRcontrol.add(gamepad.getLeftX());
            DRcontrol.add(gamepad.getLeftY());
            DRcontrol.add(gamepad.getRightX());
            DRcontrol.add(gamepad.getRightY());

            driveTrain.FieldOrientDrive(DRcontrol,false,0);
        }

        @Override
        public boolean isFinished(){
            return false;
        }
}
