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

        //PID STUFF
        public static double Kp = 0.009;
        public static double Ki = 0;
        public static double Kd = 0.01;
        public static double KiClamp = 0;
        public static double KOutClamp = 1;
        public static double KOutRateClamp = 0;
        public static double cycleRate = 10;

    }

    public static final class DriveTrain{

        //PID STUFF
        public static double Kp = 0.018;
        public static double Ki = 0;
        public static double Kd = 0.03;
        public static double KiClamp = 0;
        public static double KOutClamp = 0;

    }

    public static final class Controllers{
        //controllers dead zone compensation to prevent ghost movements
        public static final double controllerDeadZone = 0.1;

        public static double getJoyStickAngleDegree(double x, double y)
        {
            return Math.toDegrees(Math.atan2(y,x));
        }
        //converts controller output (0 to 179.999 jump to - 180 and back to 0) to bearing from 0 to 360
        public static double FTCjoystick360LEFT(double startAngle){

            if(startAngle <= -90){
                return -startAngle-90;
            }
            //if((startAngle >= 0 && startAngle < 90) || startAngle < 0)
            else{
                return 270-startAngle;
            }

        }

        //converts controller output (0 to 179.999 jump to - 180 and back to 0) to bearing from 0 to 360
        public static double FTCjoystick360RIGHT(double startAngle){

            if(startAngle >= 90){
                return startAngle-90;
            }
            //if((startAngle >= 0 && startAngle < 90) || startAngle < 0)
            else{
                return 270+startAngle;
            }

        }



    }




}
