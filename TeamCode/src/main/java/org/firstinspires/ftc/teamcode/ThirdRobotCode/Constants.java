package org.firstinspires.ftc.teamcode.ThirdRobotCode;

public class Constants {
    /**
     * All constants needed for the chassis
     * */
    class ChassisConstants{
        public static final int WHEELDIAMATER = 0; //TODO: find this number
        public static final double WHEELCIRCUMFRANCE = WHEELDIAMATER* Math.PI;
        public static final double CHASSISWIDTH = 0; //TODO: find this number
        public static final double CHASSISLENGTH = 0; // TODO: find this number

    }
    /**
     * All constants needed for the the arm
     * */
    class ArmConstants{
        public static final double START_CLIMB = 0; // TODO: find this number
        public static final double GEARRATIO = 0; //TODO: get this number
        public static final double CLAW_SERVO_START_POSITION = 0; //TODO: get this number
        public static final double WHEEL_DIAMETER = 0; //TODO: get this number
        public static final double CLIMB_UP_ANGLE = 0; //TODO: find this number
        public static final double CLIMB_DOWN_ANGLE = 0d; //TODO: find this number
        public static final double CLIMB_SPEED = 0; //TODO: find this number
    }

    static class SlideConstants {
        public static final int FORWARDLIMIT = 0; //TODO: get this number
        public static final int BACKWARDLIMIT = 0; // TODO: get this number
        public static final double GEARDIAMETER = 0; //TODO: get this number
        public static final double LIMIT = 41-Math.cos(85*Math.PI/180);
        public static final double ARM_LENGTH = 0; //TODO: get this number
        public static final double CLIMB_UP = 0; //TODO: find this number
        public static final double CLIMB_DOWN = 0;//TODO: find this number


    }
    /**
     * Auto constants needed for auto
     * */
    class AutoConstants{

    }
    /**
     * Teleop constants needed for teleop
     * */
    class TeleopConstants{

    }
}
