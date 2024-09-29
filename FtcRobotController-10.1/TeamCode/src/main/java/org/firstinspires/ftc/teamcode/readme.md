    This sections stores bunch of useful functions to remind your self when and what can be used
    
    
    
    Working with timed functions
    //time object
    private ElapsedTime runtime = new ElapsedTime();
    
    //make sure in the beginning run this reset timer
    runtime.reset();
    
    //getting time in millisecond repeatability about 1.5-2 millisecond more the good enough.
    runtime.milliseconds(); 
    
    


    Using FTCDashboard Unofficial for visual data
    //setup dashboard objects
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    dashboardTelemetry.addData("runtime", runtime.milliseconds()); // add date to dashboard, name, value
    dashboardTelemetry.update(); //updates dashboard




    Using FTCDashboard Unofficial for direct control over the code
    //creat class with @Config
    @Config
    public static class DriveTrain{
    //example of controlled variable
    public static double motor_RawPower = 0;
    }



    Periodic Functionj call, useful to call function periodic in any class that extends periodicm at specific rate
    //Create object of periodic of specified class
    PeriodicScheduler.register(driveTrain); //this is inside main in "int"
    //runs in the loop
    PeriodicScheduler.runPeriodically();//updates periodic class
    
    to add periodic function to specific class add this extends Periodic
    in the contructor add super(5000, 0);  // Run every 1000ms (1 second), no offset