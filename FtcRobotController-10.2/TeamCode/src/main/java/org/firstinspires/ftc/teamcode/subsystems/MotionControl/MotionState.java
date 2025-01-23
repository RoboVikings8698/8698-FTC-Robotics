package org.firstinspires.ftc.teamcode.subsystems.MotionControl;

public class  vMotionState {

    private double x;
    private double y;
    private double velocity;
    private double acceleration;
    private double time;

    public MotionState(double x, double y, double velocity, double acceleration, double time) {
        this.x = x;
        this.y = y;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.time = time;
    }

    @Override
    public String toString() {
        return "Position: (" + x + ", " + y + "), Velocity: " + velocity +
                ", Acceleration: " + acceleration + ", Time: " + time;
    }
}
