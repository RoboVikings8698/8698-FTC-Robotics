package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Motors extends Periodic{

    //create motor instances
    private Motor m_motor;
    private DcMotor motor;
    private Motor.Encoder encoder;
    private Controllers PID;
    public Controllers.PositionPID posPID;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    //constructor
    Motors(HardwareMap hardwareMap, String MotNam){
        //set periodic rate, 10ms
        super((long)Constants.Motors.cycleRate, 0);
        //PID stuff, skip it
        PID = new Controllers();
        posPID = PID.new PositionPID(Constants.Motors.Kp, Constants.Motors.Ki, Constants.Motors.Kd, Constants.Motors.KiClamp, Constants.Motors.KOutClamp, Constants.Motors.KOutRateClamp);



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
        if (Constants.Motors.StandbyMode == 1) {
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }else{
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }


        //reset encoder before it being used, good habit.
        m_motor.resetEncoder();
    }




    //motor Functions
    public void set(double RawPower){m_motor.set(RawPower);}

    //get motorPosition
    public int getPosition(){return encoder.getPosition();}

    //get motor velocity
    public double getVelocity(){return encoder.getCorrectedVelocity();};

    //PID controlled
    public void setPosition(double setpoint)
    {
        posPID.setNewPoint(setpoint);
    }





    @Override
    public void periodic() {
        //updating stuff
        if(Constants.DriveTrain.motorPidTrue) {
            posPID.calculatePID(getPosition());
            set(posPID.getPidOut());
        }
    }

}
