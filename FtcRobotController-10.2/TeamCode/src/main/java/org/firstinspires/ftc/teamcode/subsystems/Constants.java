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
        //Drive Train Motor Mode: 0 - coast, 1 - break
        public static final int DT_StandbyMode = 1;
        public static Object cycleRate;



        //Arm Motor
        public static final String Arm = "arm";
        public static final int Arm_pidMode = 1; //positional
        public static final double Arm_pidCycle = 35;
        public static final double kp = 0.003;
        public static final double kd = 0;

        //THE REST OF NON DRIVE TRAIN RELATED MOTORS ARE PLACED BELLOW
        //EXAMPLE FOR EACH MOTOR


        //Motor Tuning
        public static final String m_test = "arm2"; //SPECIFY MOTOR LOCATION From drive hub
        public static double m_test_cycleRate = 40; //update time pid



    }

    public static final class Servos{

        public static final String cServo1 = "intake";
        public static final String Servo1 = "yawIntake";
    }


    //variables that specify calibration values for drive train
    public static final class DriveTrain{

        //Constants
        public static int time = 10; //update loop for driveTrain

        //PID gains
        public static double Kp = 0.015;
        public static double Ki = 0;
        public static double Kd = 0.01;
        public static double KiClamp = 0;
        public static double KOutClamp = 1;

        //

    }

    public static final class Controllers{
        //controllers dead zone compensation to prevent ghost movements
        public static final double controllerDeadZone = 0.05;

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
