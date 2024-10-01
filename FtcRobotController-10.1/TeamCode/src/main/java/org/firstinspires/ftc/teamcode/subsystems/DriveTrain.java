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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    //constructor, called once object is created
    public DriveTrain(HardwareMap hardwareMap){
        super(5000, 0);  //
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

        //setup periodic for motors
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
        motor_4.set(joystick1+joystick2-joystick4);
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
    }



}
