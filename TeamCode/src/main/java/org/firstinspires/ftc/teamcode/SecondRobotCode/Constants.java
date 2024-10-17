package org.firstinspires.ftc.teamcode.SecondRobotCode;

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
    class ArmConstants {

        public static final double GEARRATIO = 1; //TODO: get this number

        double KP = 0;
        public static final double TOLERANCE = 0;//TODO: find this number
        public static final double Kp = 0; //TODO: Find P Value
        public static final double Ki = 0; //TODO: Find I Value
        public static final double Kd = 0; //TODO: Find D Value
        public static final double ENCODERTICKPERROTATION = 0; //TODO: Find the amount of ticks per rotation
        public static final double GEARREDUCTION = 0; //TODO: Find gear reduction

    }
    public static class ArmExtender {
       public static final double WHEEDIAMITER = 1; //TODO: Find wheel diameter
    }
    /**
     * Auto constants needed for auto
     * */

    class intakeConstants {
        public static final double MAX_INTAKE_POSITION = 0; //TODO: Find the Max Servo position for INTAKE
        public static final double MIN_INTAKE_POSITION = 0; //TODO: Find the MIN Servo position for INTAKE
        public static final double MAX_UP_POSITION = 0; //TODO: Find the MAX servo position for moving it UP
        public static final double MIN_DOWN_POSITION = 0; //TODO: Find the MIN servo position for moving it DOWN
        public static final double START_INTAKE__POSITION = 0; //TODO: Find the starting position for INTAKE
        public static final double START_MIDDLE_POSITION = 0; //TODO: Find the starting position for the UP/DOWN Servo
    }
    class AutoConstants{

    }
    /**
     * Teleop constants needed for teleop
     * */
    class TeleopConstants{

    }
}
