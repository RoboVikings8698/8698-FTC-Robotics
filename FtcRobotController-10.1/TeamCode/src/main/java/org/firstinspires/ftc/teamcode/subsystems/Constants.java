package org.firstinspires.ftc.teamcode.subsystems;

public final class Constants {

    //ALL MOTOR DECLARATIONS HERE
    public static final class Motors {
        public static final double MotorB5202312crp = 537.7;
        public static final double MotorB5202312rpm = 312;
        //motor mapping constants DriveTrain
        public static final String Motor1 = "motor1"; //SPECIFY MOTOR LOCATION
        public static final String Motor2 = "motor2"; //SPECIFY MOTOR LOCATION
        public static final String Motor3 = "motor3"; //SPECIFY MOTOR LOCATION
        public static final String Motor4 = "motor4"; //SPECIFY MOTOR LOCATION

        //specify if you want pid enabled or not
        public static final boolean DT_PID_Enable = false;

        //Drive Train Individual motor PID Param, to find those values proceed with tuning procedure
        public static final double DT_Kp = 0;
        public static final double DT_Ki = 0;
        public static final double DT_Kd = 0;
        public static final double DT_KiClamp = 0;
        public static final double DT_KOutClamp = 0;
        public static double DT_cycleRate = 20; //update time pid

        //Drive Train Motor Mode: 0 - coast, 1 - break
        public static final int DT_StandbyMode = 1;
        public static Object cycleRate;

        //THE REST OF NON DRIVE TRAIN RELATED MOTORS ARE PLACED BELLOW
        //EXAMPLE FOR EACH MOTOR
        /*
        public static final String MotorArm = "motorArm"; //SPECIFY MOTOR LOCATION From drive hub
        public static final boolean MotorArm_PID_Enable = false;
        public static final double MA_Kp = 0;
        public static final double MA_Ki = 0;
        public static final double MA_Kd = 0;
        public static final double MA_KiClamp = 0;
        public static final double MA_KOutClamp = 0;
        public static double MA_cycleRate = 20; //update time pid
        public static final int MA_DR_StandbyMode = 1;
         */

        public static final String MotorLift = "motorLift";
        public static final boolean MotorLift_PID_Enable = false;
        public static final double ML_Kp = 0;
        public static final double ML_Ki = 0;
        public static final double ML_Kd = 0;
        public static final double ML_KiClamp = 0;
        public static final double ML_KOutClamp = 0;
        public static double ML_cycleRate = 20; //update time pid
        public static final int ML_DR_StandbyMode = 0;

    }

    //variables that specify calibration values for drive train
    public static final class DriveTrain{

        //PID STUFF
        public static double Kp = 0.018;
        public static double Ki = 0;
        public static double Kd = 0.03;
        public static double KiClamp = 0;
        public static double KOutClamp = 1;

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
