package org.firstinspires.ftc.teamcode.teleop.Subsystem;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Functions;
import org.firstinspires.ftc.teamcode.subsystems.MotionControl.MotionControllers;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.Periodic;

import java.util.ArrayList;

public class DriveTrain  extends SubsystemBase {

    //motor declaration
    private Motors motor_1, motor_2, motor_3, motor_4;
    //gyro stuff
    private IMU imu;
    private RevHubOrientationOnRobot orientationOnRobot; //gyro stuff
    //dash board stuff
    FtcDashboard dashboard = FtcDashboard.getInstance();  //declaration dashboard
    Telemetry dashboardTelemetry = dashboard.getTelemetry(); //declaration dashboard


    //motion controllers
    private MotionControllers PID; //declare PID
    public MotionControllers.AnglePID posPID; //declare position PI// D


    //Active Variables, some used for dashboard telemetry
    double RSAngle = 0;
    private double alpha = 1; // Smoothing factor
    private double filteredYawVel = 0; // Variable to hold the filtered value
    double yawComp = 0;








    //Constructor, creates drivetrain, initializes variables and motor objects
    public DriveTrain(HardwareMap hardwareMap){



        //PID initialization//
        PID = new MotionControllers();
        //sets the default pid gains from constants, can be later updated through Dashboard variables
        posPID = PID.new AnglePID(Constants.DriveTrain.Kp, Constants.DriveTrain.Ki, Constants.DriveTrain.Kd, Constants.DriveTrain.KiClamp, Constants.DriveTrain.KOutClamp);


        //motor declaration and initialization
        motor_1 = new Motors(hardwareMap, Constants.Motors.Motor1);
        motor_1.BreakMode();
        motor_2 = new Motors(hardwareMap, Constants.Motors.Motor2);
        motor_2.BreakMode();
        motor_3 = new Motors(hardwareMap, Constants.Motors.Motor3);
        motor_3.BreakMode();
        motor_4 = new Motors(hardwareMap, Constants.Motors.Motor4);
        motor_4.BreakMode();

        //Gyro Initialization
        imu = hardwareMap.get(IMU.class, "imu");
        //mounting configuration
        double xRotation = 0;  // enter the desired X rotation angle here.
        double yRotation = 0;  // enter the desired Y rotation angle here.
        double zRotation = 90;  // enter the desired Z rotation angle here.
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        // Now initialize the IMU with this mounting orientation
        orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //setup periodic for motors, in case with want to setup velocity pid
        //PeriodicScheduler.register(motor_1);
        //PeriodicScheduler.register(motor_2);
        //PeriodicScheduler.register(motor_3);
        //PeriodicScheduler.register(motor_4);
    }







    //mot1 front left, mot2 front right, mot3 back left, mot4, back right
    public void directDrive(double joystick1, double joystick2, double joystick3, double joystick4) {
        //takes care of driving: forward-back, right-left; and clamps it so value is no more 100% of motor output
        motor_1.set(joystick1-joystick2-joystick4);
        motor_2.set(-joystick1-joystick2-joystick4);
        motor_3.set(joystick1+joystick2-joystick4);
        motor_4.set(-joystick1+joystick2-joystick4);
    }

    /* depreciated
    public void FieldOrientDrive(double LSvy, double LSvx, double RSvy, double RSvx, boolean YawOverride, double Yaw) {

        double yawComp = 0;

        //getting joystick magnitude
        double LSMagnitude = Functions.VectorMagnitude(LSvy, LSvx); //find magnitude of the vector
        double RSMagnitude = Functions.VectorMagnitude(RSvy,RSvx); //find magnitude of the vector

        //getting bearing degree from controllers
        double LSAngle = Constants.Controllers.FTCjoystick360LEFT(Constants.Controllers.getJoyStickAngleDegree(LSvx,LSvy)); //Calculating angle from vector and converting to 360 bearing
        RSAngle = Constants.Controllers.FTCjoystick360RIGHT(Constants.Controllers.getJoyStickAngleDegree(RSvx,RSvy)); //Calculating angle from vector and converting to 360 bearing


        //this code disables driver yaw input if yaw override is off, also serves as initial yaw stabilized to prevent
        //robot from jerking, since now default stating degree of joystick is in the middle is 0.
        if(YawOverride){
            posPID.setNewAngle(Yaw);//new "mission" for PID
            yawComp = posPID.getPidOut();
            RSMagnitude = 1;

        }else if(RSMagnitude == 0){
            //do nothing
        }else{
            posPID.setNewAngle(RSAngle);//new "mission" for PID
            yawComp = posPID.getPidOut(); //save calculated PId output
        }



        //motor stuff, basically, take left joystick vector break it into degrees and magnitude, compensate for motor offset and feed into motors. Also multiply pid out to prevent robot from keeping yaw position while zero feed from driver
        motor_1.set(-(LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) + LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);
        motor_2.set((LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) + LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);
        motor_3.set(-(LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) - LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);
        motor_4.set((LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) - LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);

    }
    */

    //works perfectly nothing to modify
    public void FieldOrientDrive(ArrayList<Double> DRcontrol, boolean YawOverride, double Yaw) {

        //LSX - 0, LSY - 1, RSX - 2, RSY - 3
        double LSvx = DRcontrol.get(0);
        double LSvy = DRcontrol.get(1);
        double RSvx = DRcontrol.get(2);
        double RSvy = DRcontrol.get(3);

        //double yawComp = 0;

        //getting joystick magnitude
        double LSMagnitude = Functions.VectorMagnitude(LSvy, LSvx); //find magnitude of the vector
        double RSMagnitude = Functions.VectorMagnitude(RSvy,RSvx); //find magnitude of the vector

        //getting bearing degree from controllers
        double LSAngle = Constants.Controllers.FTCjoystick360LEFT(Constants.Controllers.getJoyStickAngleDegree(LSvx,LSvy)); //Calculating angle from vector and converting to 360 bearing
        RSAngle = Constants.Controllers.FTCjoystick360RIGHT(Constants.Controllers.getJoyStickAngleDegree(-RSvx,RSvy)); //Calculating angle from vector and converting to 360 bearing


       //this code disables driver yaw input if yaw override is off, also serves as initial yaw stabilized to prevent
        //robot from jerking, since now default stating degree of joystick is in the middle is 0.
        if(YawOverride){
            posPID.setNewAngle(Yaw);//new "mission" for PID
//            posPID.setNewAngle(180); //TEST

            yawComp = posPID.getPidOut();

            RSMagnitude = 1;

        }else if(RSMagnitude == 0){
            //do nothing
        }else{
            posPID.setNewAngle(RSAngle);//new "mission" for PID

            yawComp = posPID.getPidOut(); //save calculated PId output

        }




        //motor stuff, basically, take left joystick vector break it into degrees and magnitude, compensate for motor offset and feed into motors. Also multiply pid out to prevent robot from keeping yaw position while zero feed from driver
        motor_1.set(-(LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) + LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);
        motor_2.set((LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) + LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) -  RSMagnitude * yawComp);
        motor_3.set(-(LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) - LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);
        motor_4.set((LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) - LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) -  RSMagnitude * yawComp);

    }

    public void FieldOrientedDriveAuto(double mg, double drYaw, double faceYaw){

        posPID.setNewAngle(faceYaw);//new "mission" for PID

        yawComp = posPID.getPidOut();

        //motor stuff, basically, take left joystick vector break it into degrees and magnitude, compensate for motor offset and feed into motors. Also multiply pid out to prevent robot from keeping yaw position while zero feed from driver
        motor_1.set(-(mg*Math.cos(Math.toRadians(drYaw-getYaw()))) + mg*Math.sin(Math.toRadians(drYaw-getYaw())) - yawComp);
        motor_2.set((mg*Math.cos(Math.toRadians(drYaw-getYaw()))) + mg*Math.sin(Math.toRadians(drYaw-getYaw())) -  yawComp);
        motor_3.set(-(mg*Math.cos(Math.toRadians(drYaw-getYaw()))) - mg*Math.sin(Math.toRadians(drYaw-getYaw())) - yawComp);
        motor_4.set((mg*Math.cos(Math.toRadians(drYaw-getYaw()))) - mg*Math.sin(Math.toRadians(drYaw-getYaw())) -  yawComp);

    }

    //getting robot yaw.
    public double getYaw()
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    };

    //get Yaw velocity displacement
    public double getYawVel() {
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        // Apply low-pass filter
        filteredYawVel = alpha * angularVelocity.zRotationRate + (1 - alpha) * filteredYawVel;

        return filteredYawVel;
    }


    //resetting robot yaw
    public void resetYaw(){
        imu.resetYaw();
    };










    @Override
    public void periodic() {
        //calling pid every now on
        //posPID.calculatePID(getYaw()); //calculate pid, +90 added to compensate for joystick offset from bearing
        posPID.calculatePID(getYaw()+180);
        dashboardTelemetry.addData("gro", getYaw());
        dashboardTelemetry.addData("pid vel", getYawVel());
        dashboardTelemetry.addData("pid out", posPID.getPidOut());



        dashboardTelemetry.addData("pidOUT123", yawComp);
        dashboardTelemetry.update();
    }



}
