package org.firstinspires.ftc.teamcode.teleop.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.myServo;

public class Lift extends SubsystemBase {
    Motors lift; //create motor lift
    //DashBoard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    myServo servo;

    boolean clawcontrol = false;

    public Lift(HardwareMap hardwaremap){
        lift = new Motors(hardwaremap,Constants.Motors.lift);
        servo = new myServo(hardwaremap, "servo", 0, 90);

        lift.pidEnable();
        lift.pidTune(0.01,0,0,0);
        lift.BreakMode();
    }

    public void liftHome(){
        lift.set(-500);
    }

    public void pickup(){
        lift.set(0);
    }
    public void openclaw(){
        servo.setToPos(0);
        clawcontrol = true;
    }

    public void closeclaw(){
        servo.setToPos(90);
        clawcontrol = false;
    }

    public void toggleclaw() {
        if (clawcontrol) {
            closeclaw();
        } else {
            openclaw();
        }
    }

    public void scoreLowBucket(){
        lift.set(-2500);
    }


    @Override
    public void periodic(){

       //dashboardTelemetry.addData("motorPosition", lift.getPosition());
       //dashboardTelemetry.update();
       //lift.pidEnable();
       //lift.BreakMode();

       //lift.pidTune(Dashboard.MotorTuning.kp,Dashboard.MotorTuning.ki,Dashboard.MotorTuning.kd, Dashboard.MotorTuning.kf);
       //lift.set(Dashboard.MotorTuning.SetPoint);

    }
}
