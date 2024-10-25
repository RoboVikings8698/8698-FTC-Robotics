package org.firstinspires.ftc.teamcode.subsystems;



import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.MotionControl.Point;

import java.util.ArrayList;


public class GamePad {

    private static GamepadEx controller1;
    private static GamepadEx controller2;



    //create those controllers
    public GamePad(com.qualcomm.robotcore.hardware.Gamepad controller1, com.qualcomm.robotcore.hardware.Gamepad controller2) {
        GamePad.controller1 = new GamepadEx(controller1);
        GamePad.controller2 = new GamepadEx(controller2);
    }

    public static class c1 {

        //checking for active driver inputs
        public static double getLX() {
            return controller1.getLeftX();
        }

        public static double getLY() {
            return controller1.getLeftY();

        }

        public static double getRX() {
            return controller1.getRightX();

        }

        public static double getRY() {
            return controller1.getRightY();

        }

        public static ArrayList<Double> getDriveJoy(){
            ArrayList<Double> DRcontrols = new ArrayList<>();

            //LSX - 0, LSY - 1, RSX - 2, RSY - 3
            double LSY = -Functions.DeadZone(getLY(), Constants.Controllers.controllerDeadZone);
            double LSX = Functions.DeadZone(getLX(), Constants.Controllers.controllerDeadZone); //gets each controller's inputs
            double RSY = -Functions.DeadZone(getRY(), Constants.Controllers.controllerDeadZone);
            double RSX = Functions.DeadZone(getRX(), Constants.Controllers.controllerDeadZone);

            DRcontrols.add(LSX);
            DRcontrols.add(LSY);
            DRcontrols.add(RSX);
            DRcontrols.add(RSY);



            return DRcontrols;
        }

        public static boolean getA(){
            return controller1.getButton(GamepadKeys.Button.A);
        }
        public static boolean getB(){
            return controller1.getButton(GamepadKeys.Button.B);
        }
        public static boolean getX(){
            return controller1.getButton(GamepadKeys.Button.X);
        }
        public static boolean getY(){
            return controller1.getButton(GamepadKeys.Button.Y);
        }
        public static boolean getLB(){
            return controller1.getButton(GamepadKeys.Button.LEFT_BUMPER);
        }
        public static boolean getRB(){
            return controller1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        }
        public static boolean getST(){
            return controller1.getButton(GamepadKeys.Button.START);
        }
        public static boolean getBK(){
            return controller1.getButton(GamepadKeys.Button.BACK);
        }
        public static boolean getLSK(){
            return controller1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
        }
        public static boolean getRSK(){
            return controller1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        }

        //d-pad
        public static boolean getDUP(){
            return controller1.getButton(GamepadKeys.Button.DPAD_UP);
        }
        public static boolean getDDN(){
            return controller1.getButton(GamepadKeys.Button.DPAD_DOWN);
        }
        public static boolean getDRT(){
            return controller1.getButton(GamepadKeys.Button.DPAD_RIGHT);
        }
        public static boolean getDLT(){
            return controller1.getButton(GamepadKeys.Button.DPAD_LEFT);
        }
    }

    public static class c2{
        //checking for active driver inputs
        //checking for active driver inputs
        public static double getLX(){
            return controller2.getLeftX();
        }
        public static double getLY(){
            return controller2.getLeftY();

        }
        public static double getRX(){
            return controller2.getRightX();

        }
        public static double getRY(){
            return controller2.getRightY();

        }

        public static boolean getA(){
            return controller2.getButton(GamepadKeys.Button.A);
        }
        public static boolean getB(){
            return controller2.getButton(GamepadKeys.Button.B);
        }
        public static boolean getX(){
            return controller2.getButton(GamepadKeys.Button.X);
        }
        public static boolean getY(){
            return controller2.getButton(GamepadKeys.Button.Y);
        }
        public static boolean getLB(){
            return controller2.getButton(GamepadKeys.Button.LEFT_BUMPER);
        }
        public static boolean getRB(){
            return controller2.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        }
        public static boolean getST(){
            return controller2.getButton(GamepadKeys.Button.START);
        }
        public static boolean getBK(){
            return controller2.getButton(GamepadKeys.Button.BACK);
        }
        public static boolean getLSK(){
            return controller2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
        }
        public static boolean getRSK(){
            return controller2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        }

        //d-pad
        public static boolean getDUP(){
            return controller2.getButton(GamepadKeys.Button.DPAD_UP);
        }
        public static boolean getDDN(){
            return controller2.getButton(GamepadKeys.Button.DPAD_DOWN);
        }
        public static boolean getDRT(){
            return controller2.getButton(GamepadKeys.Button.DPAD_RIGHT);
        }
        public static boolean getDLT(){
            return controller2.getButton(GamepadKeys.Button.DPAD_LEFT);
        }


    }
}
