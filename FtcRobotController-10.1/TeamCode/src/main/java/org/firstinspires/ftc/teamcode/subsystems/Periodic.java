package org.firstinspires.ftc.teamcode.subsystems;
public abstract class Periodic {
    private long lastRunTime = 0;
    private long periodMs;
    private long offsetMs;

    // Constructor for default period and offset
    public Periodic(long periodMs, long offsetMs) {
        this.periodMs = periodMs;
        this.offsetMs = offsetMs;
        this.lastRunTime = System.currentTimeMillis() + offsetMs;  // Apply the offset
    }

    // Abstract method to be implemented by subclasses
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
