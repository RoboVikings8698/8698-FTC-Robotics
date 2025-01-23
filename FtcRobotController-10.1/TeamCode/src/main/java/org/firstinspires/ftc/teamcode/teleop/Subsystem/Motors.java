package org.firstinspires.ftc.teamcode.teleop.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.MotionControl.MotionControllers;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.Periodic;


//Motors class that helps setup each motor individually
public class Motors extends SubsystemBase {

    private Motor m_motor; //motor instance
    private DcMotor motor; //motor instance
    private Motor.Encoder encoder; //encoder

    //Controllers
    private boolean pidEnabled = false;
    PIDFController pidf; // controller object
    private double setpoint;


    //DashBoard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();






    //Motor constructor setup + PID
    public Motors(HardwareMap hardwareMap, String MotNam){
        //Motor mapping, and initialization using defaults
        m_motor = new Motor(hardwareMap, MotNam, Constants.Motors.MotorB5202312crp, Constants.Motors.MotorB5202312rpm);
        DcMotor motor = m_motor.motor; //internal stuff
        encoder = m_motor.encoder; //setup encoder
        m_motor.setRunMode(Motor.RunMode.RawPower); //set raw power input
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE); //set to break mode during idle
        m_motor.resetEncoder(); //reset encoder, just a good habit


        //controller in case needed
        pidf = new PIDFController(0,0,0,0);
    }


    //set raw power directly -1 to 1
    public void set(double RawPower){
        if(pidEnabled){
            setpoint = RawPower;
        }else{
            m_motor.set(RawPower);
        }

    }

    public void encoderReset(){
        encoderReset();
    }

    public void pidEnable(){
        pidEnabled = true;
    }

    public void pidTune(double kp, double ki, double kd, double kf){
        pidf.setPIDF(kp,ki,kd,kf);
    }




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









//this will run periodically if pid is used on that particular motor

    @Override
    public void periodic(){
        //dashboardTelemetry.addData("motorPosition", getPosition());
        //dashboardTelemetry.update();
        //pidEnable();
        //BreakMode();

        //pidTune(Dashboard.MotorTuning.kp,Dashboard.MotorTuning.ki,Dashboard.MotorTuning.kd, Dashboard.MotorTuning.kf);
        //set(Dashboard.MotorTuning.SetPoint);


        //m_motor.set(pidf.calculate(getPosition(),setpoint));

    }


}
