package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Dashboard {

    @Config
    public static class DriveTrain {

        //variables for checking motors
        public static double motor_RawPower = 0;
        public static int motorNum = 1;

        public static double Kp = 0;
        public static double Ki = 0;
        public static double Kd = 0;
        public static double KiClamp = 0;
        public static double KOutClamp = 0;
        public static double KOutRateClamp = 0;
        public static double SetPoint = 0;
        public static double cycleRate = 10;



    }


}
