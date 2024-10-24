package org.firstinspires.ftc.teamcode.subsystems.SubsystemCore;

import java.util.ArrayList;
import java.util.List;

public class PeriodicScheduler {
    private static List<Periodic> periodicSystems = new ArrayList<>(); //stores classes that extends periodic

    // Add any class that extends Periodic, add them to the list
    public static void register(Periodic system) {
        periodicSystems.add(system);
    }

    // Method to trigger all periodic methods if they're ready to run
    public static void runPeriodically() {
        for (Periodic system : periodicSystems) {
            system.runPeriodicIfReady();
        }
    }
}
