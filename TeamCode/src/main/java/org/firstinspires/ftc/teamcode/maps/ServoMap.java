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
    INTAKE("intake"),
    INTAKE_X_JOINT("intake_x_joint"),
    INTAKE_Y_JOINT("intake_y_joint");

    private final String id;

    ServoMap(String id) {
        this.id = id;
    }

    public String getId() {
        return id;
    }
}
