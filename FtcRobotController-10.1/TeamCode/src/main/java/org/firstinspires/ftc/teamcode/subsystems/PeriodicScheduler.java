package org.firstinspires.ftc.teamcode.subsystems;

import java.util.ArrayList;
import java.util.List;

public class PeriodicScheduler {
    private static List<Periodic> periodicSystems = new ArrayList<>();

    // Add any class that extends Periodic
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
