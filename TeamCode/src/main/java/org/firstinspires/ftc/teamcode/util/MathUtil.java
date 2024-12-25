package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DriveConstants;

public class MathUtil {
    public static double countsToDeg(int counts, double maxCounts) {
        return ((counts * 360) / maxCounts);
    }

    public static double degToCounts(double deg, double maxCounts) {
        return ((deg * maxCounts) / 360.0f);
    }

    public static double encoderTicksToMeter(double ticks, double ticksPerRev, double radius) {
        return ((Math.PI * 2 * radius) / ticksPerRev) * ticks;
    }

    /**
     * Converts motor ticks per second to meters per second.
     *
     * @param ticksPerSecond The motor encoder ticks per second.
     * @param radius The radius of the wheel in meters.
     * @param ticksPerRev The number of encoder ticks per full revolution.
     * @return The velocity in meters per second.
     */
    public static double encoderTPSToMPS(double ticksPerSecond, double radius, double ticksPerRev) {
        return (2 * Math.PI * radius) * (ticksPerSecond / ticksPerRev);
    }

    public static double meterToInch(double meter) {
        return meter * 39.3700787;
    }

    public static double inchToMeter(double inch) {
        return inch * 0.0254;
    }
}
