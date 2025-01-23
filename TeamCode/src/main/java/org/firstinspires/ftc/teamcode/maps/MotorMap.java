package org.firstinspires.ftc.teamcode.maps;

/**
 * Contains all motors
 * <pre>
 *  {@code
 *  ARM("arm", 288, 6000),
 *  JOINT("joint", 24, 500);
 *  }
 * </pre>
 **/
public enum MotorMap {
    LEG_FRONT_RIGHT("leg_front_right", 0, 312),
    LEG_FRONT_LEFT("leg_front_left", 0, 312),
    LEG_BACK_LEFT("leg_back_left", 0, 312),
    LEG_BACK_RIGHT("leg_back_right", 0, 312),
    ELEVATOR_LEFT("elevator_left", 537.7, 312),
    ELEVATOR_RIGHT("elevator_right", 537.7, 312),
    EXTENDER("extender",	384.5, 435),
    INTAKE("intake",	312, 0);


    private final String id;
    private final double maxRPM;
    private final double ticksPerRev;

    MotorMap(String id, double ticksPerRev, double maxRPM) {
        this.id = id;
        this.maxRPM = maxRPM;
        this.ticksPerRev = ticksPerRev;
    }

    public String getId() {
        return id;
    }

    public double getMaxRPM() {
        return maxRPM;
    }

    public double getTicksPerRev() {
        return ticksPerRev;
    }
}
