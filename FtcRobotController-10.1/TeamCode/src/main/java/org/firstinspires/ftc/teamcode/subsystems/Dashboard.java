package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Dashboard {

    @Config
    public static class Motors {
        //variables for checking motors

        public static double Kp = 0.009;
        public static double Kd = 0.01;
        public static double SetPoint = 0;
    }

    @Config
    public static class DriveTrain {
        //variables for checking motors

        public static double pKp = 0;
        public static double pKd = 0;
        public static double vKp = 0;
        public static double vKd = 0;

    }




}
