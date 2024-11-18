package org.firstinspires.ftc.teamcode.subsystems.SubsystemCore;
public abstract class Periodic {
    private long lastRunTime = 0; //saves last time periodic was run
    private long periodMs;  //frequency of running
    private long offsetMs; //offset so periodic of diff calls so not interfere

    // Constructor for default period and offset, setup periodic
    public Periodic(long periodMs, long offsetMs) {
        this.periodMs = periodMs;
        this.offsetMs = offsetMs;
        this.lastRunTime = System.currentTimeMillis() + offsetMs;  // Apply the offset
    }

    // Abstract method to be implemented by subclasses, um IDK
    public abstract void periodic();

    // Method to determine if it's time to run the periodic method
    public boolean shouldRun() {
        long currentTime = System.currentTimeMillis();
        return (currentTime - lastRunTime) >= periodMs;
    }

    // Run the periodic if time threshold is met
    public void runPeriodicIfReady() {
        if (shouldRun()) {
            periodic();
            lastRunTime = System.currentTimeMillis();
        }
    }
}
