package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain  extends Periodic{

    private DriveTrainSubsystem motor_1, motor_2, motor_3, motor_4;
    private int i = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    //constructor, called once object is created, currently does nothing
    public DriveTrain(HardwareMap hardwareMap){
        super(5000, 0);  // Run every 1000ms (1 second), no offset
        //create motor objects and hardwareMap them
        motor_1 = new DriveTrainSubsystem(hardwareMap, Constants.DriveTrainSubsystemsConstants.Motor1);
        motor_2 = new DriveTrainSubsystem(hardwareMap, Constants.DriveTrainSubsystemsConstants.Motor2);
        motor_3 = new DriveTrainSubsystem(hardwareMap, Constants.DriveTrainSubsystemsConstants.Motor3);
        motor_4 = new DriveTrainSubsystem(hardwareMap, Constants.DriveTrainSubsystemsConstants.Motor4);
        //setup periodic for motors
        PeriodicScheduler.register(motor_1);
        PeriodicScheduler.register(motor_2);
        PeriodicScheduler.register(motor_3);
        PeriodicScheduler.register(motor_4);
    }







    //*********************************************************************************
    //USEFUL FUNCTIONS*****************************************************************
    //*********************************************************************************

    //this function takes driver input, converts into robot drive output
    //mot1 front left, mot2 front right, mot3 back left, mot4, back right
    void directDrive(double joystick1, double joystick2, double joystick3, double joystick4) {
        //takes care of driving: forward-back, right-left; and clamps it so value is no more 100% of motor output


        //takes driver input and relative position of the robot to the field to calculate right robot rotation angle
        FieldOrientedHeading(joystick3, joystick4);
    }


    // xposition and yposition of the right joystick
    void FieldOrientedHeading(double xpos, double ypos)
    {

    }











    /*_DEV TOOLS__________________________________________________________________________________________*/
    // Direct motor control
    public void DirectMotorControl(int motNum, double RawPower)
    {
           switch(motNum) {
                case 1:
                    motor_1.set(RawPower);
                    break;
                case 2:
                    motor_2.set(RawPower);
                    break;
                case 3:
                    motor_3.set(RawPower);
                    break;
                case 4:
                    motor_4.set(RawPower);
                    break;
                default:
           }


    }

    @Override
    public void periodic() {

        // Code that will run periodically
        dashboardTelemetry.addData("runtime", 1+i); // add date to dashboard, name, value
        dashboardTelemetry.update(); //updates dashboard
        i++;
    }



}
