package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain  extends Periodic{

    private Motors motor_1, motor_2, motor_3, motor_4;
    private int i = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    //constructor, called once object is created, currently does nothing
    public DriveTrain(HardwareMap hardwareMap){
        super(5000, 0);  // Run every 1000ms (1 second), no offset
        //create motor objects and hardwareMap them
        motor_1 = new Motors(hardwareMap, Constants.Motors.Motor1);
        motor_2 = new Motors(hardwareMap, Constants.Motors.Motor2);
        motor_3 = new Motors(hardwareMap, Constants.Motors.Motor3);
        motor_4 = new Motors(hardwareMap, Constants.Motors.Motor4);
        //setup periodic for motors
        PeriodicScheduler.register(motor_1);
        //PeriodicScheduler.register(motor_2);
        //PeriodicScheduler.register(motor_3);
        //PeriodicScheduler.register(motor_4);
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
    public void DirectMotorControl(int motNum, double RawPower, double Kp, double Ki, double Kd, double KiClamp, double KClamp, double KClampOutRate, double setpoint)
    {
           switch(motNum) {
                case 1:
                    //motor_1.set(RawPower);
                    motor_1.posPID.setPIDparams(Kp,Ki,Kd,KiClamp,KClamp,KClampOutRate);
                    motor_1.setPosition(setpoint);
                    break;
                case 2:
                    //motor_1.set(RawPower);
                    motor_2.posPID.setPIDparams(Kp,Ki,Kd,KiClamp,KClamp,KClampOutRate);
                    motor_1.setPosition(setpoint);
                    break;
                case 3:
                    //motor_1.set(RawPower);
                    motor_3.posPID.setPIDparams(Kp,Ki,Kd,KiClamp,KClamp,KClampOutRate);
                    motor_1.setPosition(setpoint);
                    break;
                case 4:
                    //motor_1.set(RawPower);
                    motor_4.posPID.setPIDparams(Kp,Ki,Kd,KiClamp,KClamp,KClampOutRate);
                    motor_1.setPosition(setpoint);
                    break;
                default:
           }


    }

    @Override
    public void periodic() {
    }



}
