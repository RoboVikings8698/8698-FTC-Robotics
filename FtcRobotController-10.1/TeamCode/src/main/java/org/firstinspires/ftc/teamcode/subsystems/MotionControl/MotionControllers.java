package org.firstinspires.ftc.teamcode.subsystems.MotionControl;
/*
 * This is PID library, made to control the position, velocity, acceleration, or all at once.
 */


import static java.lang.Math.cos;

import org.firstinspires.ftc.teamcode.subsystems.Functions;

public class MotionControllers {


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
            double error = setpoint - input;//error calculation
            accumulatedError += error; //ki calculation
            accumulatedError = Functions.Clamp(accumulatedError, -kiClamp, kiClamp); //ki clamp
            double delta_error = error - oldError; //finding derivative

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

    public class AnglePID{
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
        public AnglePID(double kp, double ki, double kd, double kiclamp, double koutClamp) {
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

    public class CascadePosVelPID{

         private PositionPID PositionControl; //cascade involves position outer loop
         private VelocityPID VelocityControl; //and velocity inner loop
            private AnglePID AngleControl; //or position is replaced wth angular control

         private double setAcceleration; //acceleration limit
         private final double velUpdateLoopFactor;
         private boolean angleControl;
         private double pidOut;
         public double desiredVelocity;
         public double poutv;
         private int i;


         //cascade constructor, set all of the tuning values
        public CascadePosVelPID(double pKp, double pKd, double vKp, double vKd, int velUpdateLoopFactor, boolean angleControl) {

            PositionControl = new PositionPID(pKp, 0, pKd, 0, 1);
            AngleControl = new AnglePID(vKp, 0, pKd, 0, 1);
            VelocityControl = new VelocityPID(pKp, 0, vKd, 0, 1);

            this.velUpdateLoopFactor = velUpdateLoopFactor;
            this.angleControl = angleControl;

        }

        // Method to update controllers based on the current state
        public void calculatePID(double currentPos, double currentVel) {

            //double desiredVelocity;

            if(velUpdateLoopFactor > i){


                //calculate positional PID, chooses between angle and position pid
               if(angleControl)
                {
                    AngleControl.calculatePID(currentPos);
                    desiredVelocity = AngleControl.getPidOut();
                    //if(AngleControl.getPidOut() < 0) {currentVelocity = currentVelocity*-1;}
                    i = 0;


                }else {
                    //PositionControl.calculatePID(currentPos);
                    //desiredVelocity = PositionControl.getPidOut();
                    //i = 0;
                    //if(PositionControl.getPidOut() < 0) {currentVelocity = currentVelocity*-1;}
                }


            }else{
                    desiredVelocity = PositionControl.getPidOut();
                    ++i;
            }



            //Apply acceleration limits
            //desiredVelocity = limitAcceleration(desiredVelocity, currentVel);

            //calculate velocity PID
            VelocityControl.calculatePID(desiredVelocity);


            //save pid output
            pidOut = VelocityControl.getPidOut();
        }

        //Acceleration limit
        private double limitAcceleration(double desiredVelocity, double currentVelocity) {

            double MAXvelRateOfChange = setAcceleration;
            double velocityChange = desiredVelocity - currentVelocity;

            // Limit the change in velocity to the maximum allowed
            if (Math.abs(velocityChange) > MAXvelRateOfChange) {
                if (velocityChange > 0) {
                    desiredVelocity = currentVelocity + MAXvelRateOfChange;
                } else {
                    desiredVelocity = currentVelocity - MAXvelRateOfChange;
                }
            }

            return desiredVelocity;
        }

        //set new target
        public void setNewTarget(double newPos, double newVel, double newAcc) {
            PositionControl.setNewPoint(newPos);
            AngleControl.setNewAngle(newPos);
            VelocityControl.setNewVel(newVel);
            this.setAcceleration = newAcc;
        }

        //pid tuning
        public void tunePID(double pKp, double pKd, double vKp, double vKd){
            PositionControl.tunePID(pKp, 0, pKd);
            AngleControl.tunePID(pKp, 0, pKd);
            VelocityControl.tunePID(vKp, 0, vKd);
        }

        public double getPIDout(){
            return pidOut;
        }

    }

    //predictable stuff can be taken care by feedforward and then any error can be compensated by PID.
    public class FeedForward {

        private int feedForward_type; // 0-default, 1-elevator, 2-arm

        private double V;  // calculated voltage
        private double Ref_velocity; // reference velocity
        private double Ref_acceleration; // reference acceleration
        private double dir; // direction (-1 or 1)
        private double theta; // arm position (angle) for use with arm control

        private double Kg; // gravitational acceleration (metric)
        private double Ks; // voltage needed to just start moving
        private double Kv; // voltage needed to maintain reference velocity
        private double Ka; // voltage needed to achieve reference acceleration

        // Constructors for different system types
        public FeedForward(double type, double kg, double ks, double kv, double ka) {
            this.feedForward_type = (int) type;
            this.Kg = kg;
            this.Ks = ks;
            this.Kv = kv;
            this.Ka = ka;
        }

        // Calculates FF assuming guessed system model is linear and returns calculated voltage
        public double calculateSimpleFF() {

            // Set the direction based on the reference velocity
            dir = (Ref_velocity < 0) ? -1 : 1;

            switch (feedForward_type) {
                case 0: // General linear system
                    V = Ks * dir + Kv * Ref_velocity + Ka * Ref_acceleration;
                    break;
                case 1: // Elevator with gravity compensation
                    V = Kg + Ks * dir + Kv * Ref_velocity + Ka * Ref_acceleration;
                    break;
                case 2: // Arm with gravity compensation based on theta (angle)
                    V = Kg * Math.cos(theta) + Ks * dir + Kv * Ref_velocity + Ka * Ref_acceleration;
                    break;
                default: // Default to general linear system
                    V = Ks * dir + Kv * Ref_velocity + Ka * Ref_acceleration;
            }

            return V;
        }

        //calculates FF assuming guessed system model is linear and returns calculated voltage
        public double calculateSimpleFF(double input){


            if(input < 0){
                dir = 1;
            }else{
                dir = -1;
            }


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

        // Placeholder for future advanced FF calculation
        public double calculateAdvancedFF(double input) {
            // In the future, implement more complex system identification models
            return V;
        }


        // Set the reference velocity
        public void setVreference(double setVar) {
            this.Ref_velocity = setVar;
        }

        // Set the reference acceleration
        public void setAreference(double setVar) {
            this.Ref_acceleration = setVar;
        }

        // Set the arm's angle (theta) for gravitational compensation in arm systems
        public void updateTheta(double setVar) {
            this.theta = setVar;
        }

    }

}

