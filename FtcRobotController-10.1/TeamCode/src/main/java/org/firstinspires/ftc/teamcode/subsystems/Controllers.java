package org.firstinspires.ftc.teamcode.subsystems;
/*
 * This is PID library, made to control the position, velocity, acceleration, or all at once.
 */


import static java.lang.Math.cos;

public class Controllers {


    //positional control PID
    public class PositionPID {

        //PID param
        private double Kp;
        private double Ki;
        private double Kd;
        private double kClampOut; //clamps pid output
        private double kiClamp; //clamps ki to prevents integral windup

        //actively used
        private double input = 0;
        public double setpoint = 0;
        private double oldError = 0;
        private double accumulatedError;
        private double pidOut;


        //constructor when new PID controller created
        public PositionPID(double kp, double ki, double kd, double kiclamp, double koutClamp) {
            Kp = kp;
            Ki = ki;
            kiClamp = kiclamp;
            Kd = kd;
            kClampOut = koutClamp;
        }


        //should be run periodically at constant rate for best results, feed current position for precision delta time can be used
        public void calculatePID(double InPut) {
            input = InPut;
            //some math
            double error = setpoint - input;
            accumulatedError += error;
            accumulatedError = Functions.Clamp(accumulatedError, -kiClamp, kiClamp);
            double delta_error = error - oldError;

            pidOut = -Kp * error + -Ki * accumulatedError + -Kd * delta_error; //main pid math
            pidOut = Functions.Clamp(pidOut, -kClampOut, kClampOut); //final result is clamped, which is basically velocity limit, voltage and speed are proportional

            oldError = error; //update var for next cycle
        }

        //new setPoint for pid to follow
        public void setNewPoint(double setPoint) {
            setpoint = setPoint;
        }

        //set new pid param
        public void tunePID(double kp, double ki, double kd){
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }


        //result of pid calculation
        public double getPidOut(){
            return pidOut;
        }
    }

    public class AngularPID{
        //PID param
        private double Kp;
        private double Ki;
        private double Kd;
        private double kClampOut; //clamps pid output
        private double kiClamp; //clamps ki to prevents integral windup

        //actively used
        private double input = 0;
        public double setAngle = 0;
        private double oldError = 0;
        private double accumulatedError;
        private double pidOut;


        //constructor when new PID controller created
        public AngularPID(double kp, double ki, double kd, double kiclamp, double koutClamp) {
            Kp = kp;
            Ki = ki;
            kiClamp = kiclamp;
            Kd = kd;
            kClampOut = koutClamp;
        }


        //should be run periodically at constant rate for best results, feed current position
        public void calculatePID(double InPut) {
            input = InPut;
            //some math
            double error = DeltaAngleDeg(input, setAngle);
            accumulatedError += error;
            accumulatedError = Functions.Clamp(accumulatedError, -kiClamp, kiClamp);
            double delta_error = error - oldError;

            pidOut = -Kp * error + -Ki * accumulatedError + -Kd * delta_error; //main pid math
            pidOut = Functions.Clamp(pidOut, -kClampOut, kClampOut); //final result is clamped, which is basically velocity limit, voltage and speed are proportional

            oldError = error; //update var for next cycle
        }

        //new setPoint for pid to follow
        public void setNewAngle(double setPoint) {
            setAngle = setPoint;
        }

        //set new pid param
        public void tunePID(double kp, double ki, double kd){
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }

        //result of pid calculation
        public double getPidOut(){
            return pidOut;
        }

        //delta angle used to calculate angle difference
        private double DeltaAngleDeg(double startAngle, double endAngle) {
            return((((endAngle - startAngle - 180) % 360) +360) % 360) - 180;
        }
    }

    public class VelocityPID{


        //PID param
        private double Kp;
        private double Ki;
        private double Kd;
        private double kClampOut; //clamps pid output
        private double kiClamp; //clamps ki to prevents integral windup

        //actively used
        private double input = 0;
        public double setpoint = 0;
        private double oldError = 0;
        private double accumulatedError;
        private double pidOut;

        //constructor when new PID controller created
        public VelocityPID(double kp, double ki, double kd, double kiclamp, double koutClamp) {
            Kp = kp;
            Ki = ki;
            kiClamp = kiclamp;
            Kd = kd;
            kClampOut = koutClamp;
        }


        //should be run periodically at constant rate for best results, feed current position
        public void calculatePID(double InPut) {
            input = InPut;
            //some math
            double error = setpoint - input;
            accumulatedError += error;
            accumulatedError = Functions.Clamp(accumulatedError, -kiClamp, kiClamp);
            double delta_error = error - oldError;

            pidOut = -Kp * error + -Ki * accumulatedError + -Kd * delta_error; //main pid math
            pidOut = Functions.Clamp(pidOut, -kClampOut, kClampOut); //final result is clamped, which is basically velocity limit, voltage and speed are proportional

            oldError = error; //update var for next cycle
        }

        //new setPoint for pid to follow
        public void setNewVel(double setPoint) {
            setpoint = setPoint;
        }

        //set new pid param
        public void tunePID(double kp, double ki, double kd){
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }

        //result of pid calculation
        public double getPidOut(){
            return pidOut;
        }
    }

    public class CascadePID{


    }

    //predictable stuff can be taken care by feedforward and then any error can be compensated by PID.
    public class FeedForward {

        private int feedForward_type = 0; //0-default, 1-elevator, 2-arm

        /*
            There are several ways to implement feedforward, guess the system assuming that the system is mostly linear,
             or creating system mathematical model that will be us to calculate feedforward.
        */


        private double V;  //calculated voltage
        private double Ref_velocity; //reference velocity
        private double Ref_acceleration; //reference acceleration
        private double dir; //direction (-1 or 1)
        private double theta; //in case of arm use, this variable with store arm position

        private double Kg; //gravitation acceleration (metric)
        private double Ks; //voltage needed to make something just to start moving
        private double Kv; //voltage need to keep reference speed
        private double Ka; //voltage needed to achieve reference acceleration

        //set FeedForward
        FeedForward(double type, double kg, double ks, double kv, double ka){
            feedForward_type = 0;
            Kg = kg;
            Ks = ks;
            Kv = kv;
            Ka = ka;
        }

        FeedForward(double type, double ks, double kv, double ka){
            feedForward_type = 0;
            Kg = 0;
            Ks = ks;
            Kv = kv;
            Ka = ka;
        }

        //calculates FF and returns calculated voltage
        public double calculateSimpleFF(){

            switch(feedForward_type){
                case 0:
                    V = Ks * dir + Kv * Ref_velocity + Ka * Ref_acceleration;
                    break;
                case 1:
                    V = Kg + Ks * dir + Kv * Ref_velocity + Ka * Ref_acceleration;
                    break;
                case 2:
                    V = Kg*cos(theta) * Ks*dir + Kv * Ref_velocity + Ka * Ref_acceleration;
                    break;

                default:
                    V = Ks * dir + Kv * Ref_velocity + Ka * Ref_acceleration;
            }

            return V;

        }
        }

}

