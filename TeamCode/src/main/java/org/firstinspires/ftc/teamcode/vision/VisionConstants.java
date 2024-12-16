package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.opencv.core.Scalar;

public class VisionConstants {
    public static final Scalar RED_LOWER1 = new Scalar(0, 120, 70);
    public static final Scalar RED_UPPER1 = new Scalar(10, 255, 255);
    public static final Scalar RED_LOWER2 = new Scalar(170, 120, 70);
    public static final Scalar RED_UPPER2 = new Scalar(180, 255, 255);

    public static final Scalar BLUE_LOWER = new Scalar(90, 120, 70);
    public static final Scalar BLUE_UPPER = new Scalar(130, 255, 255);

    public static final Scalar YELLOW_LOWER = new Scalar(15, 120, 70);
    public static final Scalar YELLOW_UPPER = new Scalar(35, 255, 255);

    public static final double MIN_AREA = 150;

    public static final double FOV_HORIZONTAL = 65;

//    public static final double KNOWN_DISTANCE_CM = 60;
//    public static final double KNOWN_WIDTH_CM = 15;

    public enum GamePieceColor {
        RED(255, 0, 0),
        BLUE(0, 0, 255),
        YELLOW(255, 255, 0);

        private final Scalar color;

        GamePieceColor(int r, int g, int b) {
            this.color = new Scalar(r, g, b);
        }

        public Scalar getColor() {
            return color;
        }

        public static GamePieceColor fromTeamColor(TeamColor teamColor) {
            switch (teamColor) {
                case BLUE:
                    return VisionConstants.GamePieceColor.BLUE;

                case RED:
                    return VisionConstants.GamePieceColor.RED;

                default:
                    return VisionConstants.GamePieceColor.YELLOW;
            }
        }
    }
}
