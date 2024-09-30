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
    private Motor.Encoder encoder;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    //variables
    private int currentPosition;






    //constructor
    DriveTrainSubsystem(HardwareMap hardwareMap, String MotNam){
        //set periodic rate
        super(10, 0);


        //motor mapping, and initialization
        m_motor = new Motor(hardwareMap, MotNam, 537.7, 312);
        //graph the internal DcMotor object
        DcMotor motor = m_motor.motor;
        //set encoder instance
        encoder = m_motor.encoder;
        //method of controlling motor, raw power, velocity feedback loop control, position feedback loop control
        //I like having full control so I choose raw power and build pid loop myself - Odysseus
        m_motor.setRunMode(Motor.RunMode.RawPower);

        //sets the motor standby mode, (break, cost)
        if (Constants.DriveTrainSubsystemsConstants.StandbyMode == 1) {
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }else{
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }
        //reset encoder before it being used, good habit.
        m_motor.resetEncoder();
    }




    //motor Functions
    public void set(double RawPower){
        m_motor.set(RawPower);
    }

    //get motorPosition
    public int getPosition(){
        return currentPosition;
    }



    //Internal functions
    public void updatePos()
    {
        currentPosition = encoder.getPosition();
    }



    @Override
    public void periodic() {
        //updating stuff
        updatePos();
    }

}
