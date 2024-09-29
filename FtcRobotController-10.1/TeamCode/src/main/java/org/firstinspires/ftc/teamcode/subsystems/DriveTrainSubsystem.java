package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrainSubsystem extends Periodic{

    //create motor instances
    private Motor m_motor;
    private DcMotor motor;
    private double currentPosition;
    private int i = 0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();




    DriveTrainSubsystem(HardwareMap hardwareMap, String MotNam){
        super(5000, 1000);  // Run every 1000ms (1 second), no offset
        //motor mapping, and initialization
        m_motor = new Motor(hardwareMap, MotNam, 537.7, 312);
        //graph the internal DcMotor object
        DcMotor motor = m_motor.motor;

        //method of controlling motor, raw power, velocity feedback loop control, position feedback loop control
        //I like having full control so I choose raw power and build pid loop myself - Odysseus
        m_motor.setRunMode(Motor.RunMode.RawPower);

        //sets the motor standby mode, done through constants
        if (Constants.DriveTrainSubsystemsConstants.StandbyMode == 1) {
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }else{
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }


    }


    //motor Functions
    public void set(double RawPower){
        m_motor.set(RawPower);
    }

    public double getPosition(){
        return currentPosition;
    }

    @Override
    public void periodic() {

        // Code that will run periodically
        dashboardTelemetry.addData("runtime", 2+i); // add date to dashboard, name, value
        dashboardTelemetry.update(); //updates dashboard
        i++;
    }

}
