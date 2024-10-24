package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MotionControl.MotionControllers;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.Periodic;

public class Motors extends Periodic {

    //create motor instances
    //motor itself
    private Motor m_motor;
    private DcMotor motor;
    //encoder stuff
    private Motor.Encoder encoder;
    //pid controller
    private MotionControllers PID;
    //position pid, also possible to use velocity pid
    public MotionControllers.PositionPID pid_p;
    public MotionControllers.VelocityPID pid_v;

    //dash board stuff
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    //constructor
    public Motors(HardwareMap hardwareMap, String MotNam, double PID_cycle, double kp, double ki, double kd, double kiclamp, double koutClamp, double M_CPR, double M_RPM, double driveMode){
        //set periodic rate, 10ms
        super((long)PID_cycle, 0);
        //PID stuff, skip it
        PID = new MotionControllers();
        pid_p = PID.new PositionPID(kp, ki, kd, kiclamp, koutClamp);
        pid_v = PID.new VelocityPID(kp, ki, kd, kiclamp, koutClamp);



        //motor mapping, and initialization
        m_motor = new Motor(hardwareMap, MotNam, M_CPR, M_RPM);
        //graph the internal DcMotor object
        DcMotor motor = m_motor.motor;
        //set encoder instance
        encoder = m_motor.encoder;
        //method of controlling motor, raw power, velocity feedback loop control, position feedback loop control
        //I like having full control so I choose raw power and build pid loop myself - Odysseus
        m_motor.setRunMode(Motor.RunMode.RawPower);

        //sets the motor standby mode, (break, cost)
        if (driveMode == 1) {
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }else{
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }


        //reset encoder before it being used, good habit.
        m_motor.resetEncoder();
    }
    //constructor

    public Motors(HardwareMap hardwareMap, String MotNam, double M_CPR, double M_RPM, double driveMode){
        super(1000000, 0);
        //set periodic rate, 10ms


        //motor mapping, and initialization //standard 537.7, 312
        m_motor = new Motor(hardwareMap, MotNam, M_CPR, M_RPM);
        //graph the internal DcMotor object
        DcMotor motor = m_motor.motor;
        //set encoder instance
        encoder = m_motor.encoder;
        //method of controlling motor, raw power, velocity feedback loop control, position feedback loop control
        //I like having full control so I choose raw power and build pid loop myself - Odysseus
        m_motor.setRunMode(Motor.RunMode.RawPower);

        //sets the motor standby mode, (break, cost)
        if (driveMode == 1) {
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }else{
            m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }


        //reset encoder before it being used, good habit.
        m_motor.resetEncoder();
    }

    //motor Functions, from -1 to 1.
    public void set(double RawPower){m_motor.set(RawPower);}

    //get motorPosition, requires encoder
    public int getPosition(){return encoder.getPosition();}

    //get motor velocity, requires encoder
    public double getVelocity(){return encoder.getCorrectedVelocity();};

    //PID controlled
    public void setPosition(double setpoint)
    {
        pid_p.setNewPoint(setpoint);
        pid_v.setNewVel(setpoint);

    }




//this will run periodically if pid is used on that particular motor
    @Override
    public void periodic() {

        if(Constants.Motors.DT_PID_Enable) {
            pid_p.calculatePID(getPosition());
            set(pid_p.getPidOut());
       }
    }

}
