package org.firstinspires.ftc.teamcode.subsystems;
/*
 * This is PID library, made to control the position, velocity, acceleration, or all at once.
 */


public class Controllers {

    //positional control PID
    public class PositionPID {

        //PID param
        private double Kp;
        private double Ki;
        private double Kd;
        private double kClampOut; //clamps pid output
        private double kiClamp; //clamps ki to prevents integral windup
        //basically velocity limit
        private double kClampOutRate = 0; //limits pid output rate of change

        //actively used
        private double input = 0;
        public double setpoint = 0;
        private double oldError = 0;
        private double accumulatedError;
        private double oldPidOut;
        private double pidOut;


        //constructor when new PID controller created
        public PositionPID(double kp, double ki, double kd, double kiclamp, double koutClamp, double kclampoutrate) {
            Kp = kp;
            Ki = ki;
            kiClamp = kiclamp;
            Kd = kd;
            kClampOut = koutClamp;
            kClampOutRate = kclampoutrate;
        }

        //constructor when new PID controller created
        public PositionPID(double kp, double ki, double kd, double kiclamp, double koutClamp) {
            Kp = kp;
            Ki = ki;
            kiClamp = kiclamp;
            Kd = kd;
            kClampOut = koutClamp;
        }


        //should be run periodically at constant rate for best results
        public void calculatePID(double InPut) {
            input = InPut;
            //some math
            double error = setpoint - input;
            accumulatedError += error;
            accumulatedError = Functions.Clamp(accumulatedError, -kiClamp, kiClamp);
            double delta_error = error - oldError;

            pidOut = -Kp * error + -Ki * accumulatedError + -Kd * delta_error; //main pid math
            pidOut = Functions.Clamp(pidOut, -kClampOut, kClampOut); //final result is clamped, which is basically velocity limit, voltage and speed are proportional
            if (kClampOutRate > 0) {
                pidOut = Functions.Clamp(-oldPidOut + pidOut, -kClampOutRate, kClampOutRate);
            }//limits acceleration, if specified
            oldPidOut = pidOut; //update var for next cycle
            oldError = error; //update var for next cycle
        }

        //new setpoint for pid to follow
        public void setNewPoint(double setPoint) {
            setpoint = setPoint;
        }


        //set new pid param
        public void setPIDparams(double kp,double ki,double kd,double kiclamp, double kOut, double kclampoutrate){
            Kp = kp;
            Ki = ki;
            Kd = kd;
            kClampOut = kOut;
            kiClamp = kiclamp;
            kClampOutRate = kclampoutrate;
        }

        public void setPidOut(double kclampout){
            kClampOut = kclampout;
        }


        //result of pid calculation
        public double getPidOut(){
            return pidOut;
        }


    }


    public class VelocityPID{


    }

    public class CascadePID{

    }


}

