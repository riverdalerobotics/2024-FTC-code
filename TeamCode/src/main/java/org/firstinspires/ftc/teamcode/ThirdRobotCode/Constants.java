package org.firstinspires.ftc.teamcode.ThirdRobotCode;

public class Constants {
    /**
     * All constants needed for the chassis
     * */
    static class ChassisConstants{
        public static final int WHEELDIAMATER = 0; //TODO: find this number
        public static final double WHEEL_CIRCUMFERENCE = WHEELDIAMATER* Math.PI;
        public static final double CHASSIS_WIDTH = 0; //TODO: find this number
        public static final double CHASSIS_LENGTH = 0; // TODO: find this number


    }
    /**
     * All constants needed for the the arm
     * */
    static class ArmConstants{
        public static final double START_CLIMB = 0; // TODO: find this number
        public static final double GEAR_RATIO = 0; //TODO: get this number
        public static final double CLAW_SERVO_START_POSITION = 0; //TODO: get this number
        public static final double WHEEL_DIAMETER = 0; //TODO: get this number
        public static final double CLIMB_UP_ANGLE = 0; //TODO: find this number
        public static final double CLIMB_DOWN_ANGLE = 0d; //TODO: find this number
        public static final double CLIMB_SPEED = 0; //TODO: find this number
        public static final double SCORE_SPEED = 0;//TODO: find this number
        public static final double BUCKET_ANGLE = 0;//TODO: find this number
        public static final double INTAKE_ANGLE = 0;//TODO: find this number
        public static final double INTAKE_SPEED = 0;//TODO: find this number

    }
    static class SlideConstants {
        public static final double INTAKE_MIN = 0;//TODO: find this number
        public static final double INTAKE_MAX = 0;//TODO: find this number
        public static final double TOLERANCE = 0;//TODO: find this number
        public static final double kp = 0;//TODO: find this number
        public static final double ki = 0;//TODO: find this number
        public static final double kd = 0;//TODO: find this number
        public static final double kf = 0;//TODO: find this number
        public static final int SPEED = 0;//TODO: find this number
        public static final double ABSOLUTE_LIMIT = 0;//TODO: find this number
        public static final int FORWARD_LIMIT = 0; //TODO: get this number
        public static final int BACKWARD_LIMIT = 0; // TODO: get this number
        public static final double GEARDIAMETER = 0; //TODO: get this number
        public static final double LIMIT = 41-Math.cos(85*Math.PI/180);
        public static final double ARM_LENGTH = 0; //TODO: get this number
        public static final double CLIMB_UP = 0; //TODO: find this number
        public static final double CLIMB_DOWN = 0;//TODO: find this number
        public static final double SCORE_BUCKET = 0;//TODO: find this number
        public static final double INTAKE_POSITION = 0;//TODO: find this number
        public static final double INTAKE_SPEED = 0; //TODO: find this number
    }
    class IntakeConstants{
        public static final long WAIT_TIME=0;//TODO: find this number
        public static final double SCORE_POSITION = 0;//TODO: find this number
        public static final double SCORE_SPEED = 0;//TODO: find this number
        public static final double INTAKE_POSITION = 0;//TODO: find this number
        public static final double INTAKE_SPEED = 0;//TODO: find this number
    }
    /**
     * Auto constants needed for auto
     * */
    static class AutoConstants{
        public static final double[] SCORE_POS = {0,0,0};
    }
    /**
     * Teleop constants needed for teleop
     * */
    class TeleopConstants{

    }
}
