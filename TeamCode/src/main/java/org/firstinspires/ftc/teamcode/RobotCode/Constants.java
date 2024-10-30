package org.firstinspires.ftc.teamcode.RobotCode;

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


        public static final double GEARRATIO = 0; //TODO: get this number

        public static final double BUCKET_ANGLE = 0; //TODO: find angle
        public static final double BAR_ANGLE = 0; //TODO: find angle
        public static final double INTAKE_ANGLE = 0; //TODO: find angle

    }

    /**
     * All constants needed for the intake
     */
    //unsure if this is needed - EMMANUEL
    class IntakeConstants{

        public static final double INTAKE_SAMPLE = 0; //TODO: find angle needed for intaking sample
        public static final double RELEASE_SAMPLE = 0; // TODO: find angle needed for releasing sample
        public static final double RESET_POSITION = 0; // TODO: find default angle of intake (if needed)

    }
    /**
     * Auto constants needed for auto
     * */
    class AutoConstants{

        //TODO: Change these Names Later (Currently just a place holder) - Emmanuel
        public static final double DISTANCE_TO_AUTO_SAMPLE_1 = 0; // TODO: find distance to the first sample

    }
    /**
     * Teleop constants needed for teleop
     * */
    class TeleopConstants{

    }
}
