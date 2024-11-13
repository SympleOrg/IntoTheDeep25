package org.firstinspires.ftc.teamcode.subsystems.elevator;

import org.firstinspires.ftc.teamcode.maps.MotorMap;

public class ElevatorConstants {
    public static final double KG = 0;

    public static final double GEAR_RATIO = 1;
    public static final double WHEEL_RADIUS = 0;

    public static final double METERS_PER_REV = (Math.PI * 2) * WHEEL_RADIUS;
    public static final double METERS_PER_TICK = (METERS_PER_REV / (MotorMap.ELEVATOR_LEFT.getTicksPerRev() * GEAR_RATIO));
}
