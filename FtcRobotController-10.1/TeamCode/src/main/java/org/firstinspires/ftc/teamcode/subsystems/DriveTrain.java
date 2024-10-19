package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveTrain  extends Periodic{

    private Motors motor_1, motor_2, motor_3, motor_4;
    private IMU imu; //gyro stuff
    private RevHubOrientationOnRobot orientationOnRobot;

    private Controllers PID; //declare PID
    public Controllers.AngularPID posPID; //declare position PID

    FtcDashboard dashboard = FtcDashboard.getInstance();  //declaration dashboard
    Telemetry dashboardTelemetry = dashboard.getTelemetry(); //declaration dashboard

    //Var
    double RSAngle; //for printing values into Dashboard
    double LSAngle; //for printing values into Dashboard
    double LSMagnitude;





    //constructor, called once object is created
    public DriveTrain(HardwareMap hardwareMap){
        super(10, 5);  //

        //pid initialization stuff
        PID = new Controllers();
        posPID = PID.new AngularPID(Constants.DriveTrain.Kp, 0, Constants.DriveTrain.Kd, 0, 1);

        //create motor objects and hardwareMap them
        motor_1 = new Motors(hardwareMap, Constants.Motors.Motor1);
        motor_2 = new Motors(hardwareMap, Constants.Motors.Motor2);
        motor_3 = new Motors(hardwareMap, Constants.Motors.Motor3);
        motor_4 = new Motors(hardwareMap, Constants.Motors.Motor4);

        //GYRO STUFF
        imu = hardwareMap.get(IMU.class, "imu");
        //mounting configuration
        double xRotation = 0;  // enter the desired X rotation angle here.
        double yRotation = 0;  // enter the desired Y rotation angle here.
        double zRotation = 0;  // enter the desired Z rotation angle here.
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
        // Now initialize the IMU with this mounting orientation
        orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //setup periodic for motors, in case with want to setup velocity pid
        PeriodicScheduler.register(motor_1);
        PeriodicScheduler.register(motor_2);
        PeriodicScheduler.register(motor_3);
        PeriodicScheduler.register(motor_4);
    }







    //mot1 front left, mot2 front right, mot3 back left, mot4, back right
    public void directDrive(double joystick1, double joystick2, double joystick3, double joystick4) {
        //takes care of driving: forward-back, right-left; and clamps it so value is no more 100% of motor output
        motor_1.set(joystick1-joystick2-joystick4);
        motor_2.set(-joystick1-joystick2-joystick4);
        motor_3.set(joystick1+joystick2-joystick4);
        motor_4.set(-joystick1+joystick2-joystick4);
    }

    public void FieldOrientDrive(double LSvy, double LSvx, double RSvy, double RSvx) {

        //getting bearing degree from controllers
        LSAngle = Constants.Controllers.FTCjoystick360LEFT(Constants.Controllers.getJoyStickAngleDegree(LSvx,LSvy)); //Calculating angle from vector and converting to 360 bearing
        RSAngle = Constants.Controllers.FTCjoystick360RIGHT(Constants.Controllers.getJoyStickAngleDegree(RSvx,RSvy)); //Calculating angle from vector and converting to 360 bearing

        //getting joystick magnitude
        LSMagnitude = Functions.VectorMagnitude(LSvy, LSvx); //find magnitude of the vector
        double RSMagnitude = Functions.VectorMagnitude(RSvy,RSvx); //find magnitude of the vector


        posPID.setNewAngle(RSAngle);//new "mission" for PID
        double yawComp = posPID.getPidOut(); //save calculated PId output


        //motor stuff, basically, take left joystick vector break it into degrees and magnitude, compensate for motor offset and feed into motors. Also multiply pid out to prevent robot from keeping yaw position while zero feed from driver
        motor_1.set(-(LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) + LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);
        motor_2.set((LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) + LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);
        motor_3.set(-(LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) - LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);
        motor_4.set((LSMagnitude*Math.cos(Math.toRadians(LSAngle-getYaw()))) - LSMagnitude*Math.sin(Math.toRadians(LSAngle-getYaw())) - RSMagnitude * yawComp);

    }


    //getting robot yaw.
    public double getYaw()
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    };

    //resetting robot yaw
    public void resetYaw(){
        imu.resetYaw();
    };










    @Override
    public void periodic() {
        //calling pid every now on
        posPID.calculatePID(getYaw()); //calculate pid, +90 added to compensate for joystick offset from bearing

        posPID.tunePID(Dashboard.DriveTrain.Kp,0,Dashboard.DriveTrain.Kd);
        dashboardTelemetry.addData("RSA", LSMagnitude);
        dashboardTelemetry.update();
    }



}
