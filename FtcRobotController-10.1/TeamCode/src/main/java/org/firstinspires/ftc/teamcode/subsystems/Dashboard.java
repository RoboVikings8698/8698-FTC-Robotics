package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Dashboard {


    @Config
    public static class MotorTuning{
        //variables to tune motor
        public static double SetPoint = 0;

        //0-direct, 1-position, 2-velocity, 3-cascade
        public static int ControlMethod = 0;
        public static double kp = 0;
        public static double ki = 0;
        public static double kd = 0;
        public static double kf = 0;
    }

    @Config
    public static class DriveTrain {
        //variables for checking motors

        public static double pKp = 0;
        public static double pKd = 0;
        public static double vKp = 0;
        public static double vKd = 0;
    }

    @Config
    public static class Commands{
        public static double YawIntakePos = 0;
    }




}
