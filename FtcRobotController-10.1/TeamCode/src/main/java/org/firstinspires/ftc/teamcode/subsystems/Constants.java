package org.firstinspires.ftc.teamcode.subsystems;

public final class Constants {
    public static final class Motors {
        //motor mapping constants
        public static final String Motor1 = "motor1"; //SPECIFY MOTOR LOCATION
        public static final String Motor2 = "motor2"; //SPECIFY MOTOR LOCATION
        public static final String Motor3 = "motor3"; //SPECIFY MOTOR LOCATION
        public static final String Motor4 = "motor4"; //SPECIFY MOTOR LOCATION

        //when motor on standby: 0 - coast, 1 - break, 2 - hold
        public static final int StandbyMode = 1;

        //Driving Mode, true for field oriented control
        public static final boolean DriveMode = true;

        //PID STUFF
        public static double Kp = 0;
        public static double Ki = 0;
        public static double Kd = 0;
        public static double KiClamp = 0;
        public static double KOutClamp = 0;
        public static double KOutRateClamp = 0;
        public static double cycleRate = 10;

    }




}
