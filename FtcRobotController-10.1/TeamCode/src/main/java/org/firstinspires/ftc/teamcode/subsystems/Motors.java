package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MotionControl.MotionControllers;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.Periodic;


//Motors class that helps setup each motor individually
public class Motors extends Periodic {

    //create motor instances
    private Motor m_motor; //motor instance
    private DcMotor motor; //motor instance

    //Encoder
    private Motor.Encoder encoder;


    //Motion Controller
    private MotionControllers PID;
    private int controlMode;
    private boolean MotionControlEnable = false;

    //Position and velocity controller
    private MotionControllers.PositionPID pid_p;
    private MotionControllers.VelocityPID pid_v;

    //DashBoard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();






    //Motor constructor setup + PID
    public Motors(HardwareMap hardwareMap, String MotNam, double PID_cycle, double kp, double ki, double kd, double kiclamp, double koutClamp, int controlmode, double M_CPR, double M_RPM){

        //set periodic rate, 10ms
        super((long)PID_cycle, 0);

        //PID stuff
        PID = new MotionControllers();
        pid_p = PID.new PositionPID(kp, ki, kd, kiclamp, koutClamp);
        pid_v = PID.new VelocityPID(kp, ki, kd, kiclamp, koutClamp);
        //mode, velocity control, position control, cascade control
        controlMode = controlmode;
        MotionControlEnable = true;



        //motor mapping, and initialization
        m_motor = new Motor(hardwareMap, MotNam, M_CPR, M_RPM);
        //graph the internal DcMotor object
        DcMotor motor = m_motor.motor;
        //set encoder instance
        encoder = m_motor.encoder;
        //method of controlling motor, raw power, velocity feedback loop control, position feedback loop control
        //I like having full control so I choose raw power and build pid loop myself - Odysseus
        m_motor.setRunMode(Motor.RunMode.RawPower);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE); //stopping mode


        //reset encoder before it being used, good habit.
        m_motor.resetEncoder();
    }


    //Motor constructor setup simplified
    public Motors(HardwareMap hardwareMap, String MotNam, double M_CPR, double M_RPM){
        //in this case it is useless
        super(10000000, 0);


        //motor mapping, and initialization //standard 537.7, 312
        m_motor = new Motor(hardwareMap, MotNam, M_CPR, M_RPM);
        //graph the internal DcMotor object
        DcMotor motor = m_motor.motor;
        //set encoder instance
        encoder = m_motor.encoder;
        //method of controlling motor, raw power, velocity feedback loop control, position feedback loop control
        //I like having full control so I choose raw power and build pid loop myself - Odysseus
        m_motor.setRunMode(Motor.RunMode.RawPower);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);//stopping mode


        //reset encoder before it being used, good habit.
        m_motor.resetEncoder();
    }



    //Main Motor Functions


    //set raw power directly -1 to 1
    public void set(double RawPower){m_motor.set(RawPower);}

    //get motorPosition, requires encoder
    public int getPosition(){return encoder.getPosition();}

    //get motor velocity, requires encoder
    public double getVelocity(){return encoder.getCorrectedVelocity();};

    //set stop mode when motor is not in use, by default it is coast
    public void BreakMode(){
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void CoastMode(){
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }






    //PID controlled

    //set point
    public void setPoint(double setpoint)
    {
        pid_p.setNewPoint(setpoint);
        pid_v.setNewVel(setpoint);
    }

    //tune motion controller
    public void tuneMotionController(double kp,double kd){
        pid_p.tunePID(kp, 0, kd);
        pid_v.tunePID(kp, 0, kd);
    }

    //set control method
    public void setControlMethod(int i){
        controlMode = i;
    }



//this will run periodically if pid is used on that particular motor
    @Override
    public void periodic() {

        //check if PID mode is enabled
        if(MotionControlEnable) {

            pid_v.calculatePID(getVelocity());
            pid_p.calculatePID(getPosition());

            switch(controlMode){
                case 1:
                    set(pid_p.getPidOut());

                    break;
                case 2:
                    set(pid_v.getPidOut());

                    break;
                case 3:

                    break;
                default:
            }

       }
    }



}
