Brief code organization

Subsystem Folder - Used for storing functions, commands, variables, etc.

    Constants - Localized Collection of variables that never change in code, but can easily changed while programming
    Functions - Localized Collection of functions that can be used through out the robot code
    DashBoard - Localized Collection of Variables that can be controlled through user interface
    DriveTrain - Responsible for controlling DriveTrain, also getting data from gryo
    DriveTrainSubsystem - Controls each motor individually which is later used by DriveTrain Class, just makes DriveTrain code cleaner
    Periodic - function that will be use as extension to creat timed functions
    PeriodicScheduler - responsible for running created functions at specific timed rate, keeps track of periodic.