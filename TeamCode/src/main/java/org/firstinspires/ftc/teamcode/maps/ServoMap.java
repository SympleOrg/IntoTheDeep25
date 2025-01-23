package org.firstinspires.ftc.teamcode.maps;

/**
 * Contains all servos
 * <pre>
 *  {@code
 *  CLAW("claw"),
 *  DRONE("drone");
 *  }
 * </pre>
 **/
public enum ServoMap {
    CLAW("claw"),
    SCORER("scorer"),
    INTAKE_JOINT("intake_joint");

    private final String id;

    ServoMap(String id) {
        this.id = id;
    }

    public String getId() {
        return id;
    }
}
