package org.firstinspires.ftc.teamcode.subsystems.extender;

public class ExternderConstants {
    public static final double Kp = 0;
    public static final double Ki = 0;
    public static final double Kd = 0;

    public static final double TICKS_PER_REV = 2000;

    public enum ExtenderState {
        CLOSE(0),
        OPEN(30);

        private final double deg;

        ExtenderState(double deg) {
            this.deg = deg;
        }

        public double getDeg() {
            return deg;
        }
    }
}
