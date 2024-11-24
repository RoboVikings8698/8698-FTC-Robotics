package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Commanding;
import org.firstinspires.ftc.teamcode.subsystems.GamePad;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCore.PeriodicScheduler;


//Main code
@TeleOp
public class MainTeleop extends OpMode {

    //DashBoard
    FtcDashboard dashboard = FtcDashboard.getInstance();  //declaration dashboard
    Telemetry dashboardTelemetry = dashboard.getTelemetry(); //declaration dashboard

    private Commanding command;



    //When Int pressed...
    @Override
    public void init() {
        PeriodicScheduler scheduler = new PeriodicScheduler();//creates periodic scheduler object
        command = new Commanding(hardwareMap, scheduler, gamepad1, gamepad2); //pass important objects to Commanding class
    }

    //Loop until start pressed
    @Override
    public void init_loop() {
        command.Home();
    }

    //When start pressed...
    @Override
    public void start() {




    }



    //While robot code is not stopped...
    @Override
    public void loop() {

        //periodic
        PeriodicScheduler.runPeriodically();
        //runs all of the subsystems and commands
        command.CommandingRun();


    }

    //Once robot stopped...
    @Override
    public void stop() {
    }

}



