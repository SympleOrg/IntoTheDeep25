package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.maps.MotorMap;

import java.util.HashMap;
import java.util.Map;

public class RobotConstants {
    public static class DriveConstants {
        public static final double TICKS_PER_REV = 2000;

        public static final double GEAR_RATIO = 1;
        public static final double WHEEL_RADIUS = 0.045;
        public static final double WHEELS_DISTANCE = 0.19;

        public static final double METERS_PER_REV = (Math.PI * 2) * WHEEL_RADIUS;
        public static final double METERS_PER_TICK = (METERS_PER_REV / (TICKS_PER_REV * GEAR_RATIO));

        public static final double Ks = 0;

        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        public enum DriveSpeed {
            NORMAL(1),
            SLOW(0.65);

            private final double maxSpeed;

            DriveSpeed(double maxSpeed) {
                this.maxSpeed = maxSpeed;
            }

            public double getMaxSpeed() {
                return maxSpeed;
            }
        }
    }

    @Config
    public static class ElevatorConstants {
        public static double KG = 0.1;

        public static double P = 15;
        public static double I = 0;
        public static double D = 0;

        public static double RESET_POWER = -1;

        public static double MAX_HEIGHT = 0.94;
        public static double MIN_HEIGHT = -0.05;

        public static double SCORE_OFFSET = -0.1;

        public static final double GEAR_RATIO = 1;
        public static final double WHEEL_RADIUS = 0.0382 * 0.5; // cry about it

        public static final double METERS_PER_REV = (Math.PI * 2) * WHEEL_RADIUS;
        public static final double METERS_PER_TICK = (METERS_PER_REV / (MotorMap.ELEVATOR_LEFT.getTicksPerRev() * GEAR_RATIO));

        public enum ElevatorState {
            BASKET_TOP(0.86),
            BASKET_BOTTOM(0.32),
            HUMAN_PLAYER(0),
            SCORE_TOP(0.45),
            SCORE_BOTTOM(0.15),
            REST(0);


            private final double meters;

            ElevatorState(double meters) {
                this.meters = meters;
            }

            public double getMeters() {
                return meters;
            }
        }
    }

    @Config
    public static class ExtenderConstants {
        public static double Kp = 10;
        public static double Ki = 0;
        public static double Kd = 0;

        public static final double MIN_POS = 0;
        public static final double MAX_POS = 0.215;

        public static final double WHEEL_RADIUS = 0.0191;
        public static final double METER_PER_REV = (2 * Math.PI * WHEEL_RADIUS);
        public static final double METER_PER_TICK = METER_PER_REV / MotorMap.EXTENDER.getTicksPerRev();
    }

    public static class ClawConstants {

        public enum ClawState {
            OPEN(15),
            CLOSE(60);

            private final double deg;

            ClawState(double deg) {
                this.deg = deg;
            }

            public double getDeg() {
                return deg;
            }
        }
    }

    public static class IntakeConstants {

        public enum IntakeState {
            OPEN(0),
            CLOSE(130);

            private final double deg;

            IntakeState(double deg) {
                this.deg = deg;
            }

            public double getDeg() {
                return deg;
            }
        }
    }

    public static class ScorerConstants {
        public enum ScorerState {
            SCORE(80),
            TAKE(0);

            private final double deg;

            ScorerState(double deg) {
                this.deg = deg;
            }

            public double getDeg() {
                return deg;
            }
        }
    }

    public static class IntakeJointConstants {
        public enum JointPitchState {
            TAKE(25),
            HUMAN_PLAYER(120),
            SAFE_PLACE(181),
            BASKET(270);

            private final double deg;

            JointPitchState(double deg) {
                this.deg = deg;
            }

            public double getDeg() {
                return deg;
            }
        }

        public enum JointRollState {
            CW_90(0),
            CW_45(1),
            ZERO(2),
            C_45(3),
            C_90(4);

            private static final Map<Integer, JointRollState> POSITIONS = new HashMap<>();

            static {
                for (JointRollState state : JointRollState.values()) {
                    POSITIONS.put(state.index, state);
                }
            }

            private final double OFFSET = 50;
            private final double JUMP = 49; // in deg

            private final int index;

            JointRollState(int index) {
                this.index = index;
            }

            public double getDeg() {
                return index * JUMP + OFFSET;
            }

            public JointRollState getPrevious() {
                return POSITIONS.getOrDefault(index - 1, this);
            }

            public JointRollState getNext() {
                return POSITIONS.getOrDefault(index + 1, this);
            }
        }
    }
}
