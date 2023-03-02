// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.ArmPosition;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public class PWM {
        public static final int BLINKIN_ID = 0;
    }

    public class DIO {
        public static final int ARM_STAGE_1_ENCODER_ID = 7;
        public static final int ARM_STAGE_2_ENCODER_ID = 8;
        public static final int ARM_STAGE_3_ENCODER_ID = 9;
    }

    public class CAN {
        //CAN Bus IDs
        public static final int PDH_ID = 1;
        public static final int PCH_ID = 2;
        public static final int PIGEON_ID = 3;

        public static final int FL_CANCODER_ID = 4;
        public static final int FR_CANCODER_ID = 5;
        public static final int BL_CANCODER_ID = 6;
        public static final int BR_CANCODER_ID = 7;

        public static final int FL_DRIVE_ID = 11;
        public static final int FR_DRIVE_ID = 12;
        public static final int BL_DRIVE_ID = 13;
        public static final int BR_DRIVE_ID = 14;

        public static final int SHWERVE_DRIVE_ID = 15;

        public static final int FL_AZIMUTH_ID = 21;
        public static final int FR_AZIMUTH_ID = 22;
        public static final int BL_AZIMUTH_ID = 23;
        public static final int BR_AZIMUTH_ID = 24;

        public static final int ARM_STAGE_1_ID = 31;
        public static final int ARM_STAGE_1_FOLLOWER_ID = 32;
        public static final int ARM_STAGE_2_ID = 33;
        public static final int ARM_STAGE_3_ID = 34;

        public static final int GRIP_LEFT_ID = 41;
        public static final int GRIP_RIGHT_ID = 42;
    }

    public class DRIVETRAIN {
        // robot width (meters)
        public static final double ROBOT_WIDTH = 0.6858;
        // wheel diameter (meters)
        public static final double WHEEL_DIAMETER = 0.1016;
        // drive gear ratio
        public static final double DRIVE_GEAR_RATIO = 6.75;

        // encoder offsets (degrees)
        public static final double FL_ECODER_OFFSET = -313.682+2;
        public static final double FR_ECODER_OFFSET = -166.553+90+2;
        public static final double BL_ECODER_OFFSET = -246.006-45-2-3;
        public static final double BR_ECODER_OFFSET = -204.258-45-2-3;
        
        /** maximum strafe speed (meters per second) */
        public static final double MAX_LINEAR_SPEED = 5.4;
        /** maximum rotation speed (radians per second) */
        public static final double MAX_ROTATION_SPEED = Math.PI*2;
        public static final double SWERVE_SLOW_SPEED_PERCENTAGE = 0.25;
            
        // pid values
        public static final double AZIMUTH_kP = 0.65;
        public static final double AZIMUTH_kD = 0.1;

        // calculated via JVN calculator
        public static final double DRIVE_kP = 0.044057;
        public static final double DRIVE_kF = 0.028998;

        /* Maximum distance for a valid waypoint (meters) */
        public static final double MAX_WAYPOINT_DISTANCE = 0.5;

        public static final double SHWERVE_DRIVE_Kp = 0;
        public static final double SHWERVE_DRIVE_Kd = 0;

        public static final double AUTO_BALANCE_Kp = 0;
        public static final double AUTO_BALANCE_Kd = 0;
    }

    public class LL {
        public static final double SLOPE = 0;
        public static final double YINT = 0;
    }

    public static final class ARM {
        public static final double JOINT_ANGLE_DEADZONE = 0;
        public static final double JOINT_COORDINATE_DEADZONE = 0;

        public static enum positions {
            ScoreHigh,
            ScoreMid,
            ScoreLow,
            Floor,
            FloorAlt,
            Substation,
            Idle
        };

        public static ArmPosition scoreHighPosition  = new ArmPosition(0, 0, 0);
        public static ArmPosition scoreMidPosition   = new ArmPosition(0, 0, 0);
        public static ArmPosition scoreLowPosition   = new ArmPosition(0, 0, 0);
        public static ArmPosition floorPosition      = new ArmPosition(0, 0, 0);
        public static ArmPosition floorAltPosition   = new ArmPosition(0, 0, 0);
        public static ArmPosition substationPosition = new ArmPosition(0, 0, 0);
        public static ArmPosition idlePosition       = new ArmPosition(154, 216, 248);

        public static final double thetaSpeed = 0.1;
        public static final double xSpeed = 0.1;
        public static final double ySpeed = 0.1;

        public static final double STAGE_1_LENGTH = 10;
        public static final double STAGE_2_LENGTH = 10;

        public static final double STAGE_1_Kp = 0.01;
        public static final double STAGE_1_Ki = 0;
        public static final double STAGE_1_Kd = 0;
        public static final double STAGE_1_Kf = 0.05;
        public static final double STAGE_2_Kp = 0.001;
        public static final double STAGE_2_Ki = 0;
        public static final double STAGE_2_Kd = 0;
        public static final double STAGE_2_Kf = 0.05;
        public static final double STAGE_3_Kp = 0.001;
        public static final double STAGE_3_Ki = 0;
        public static final double STAGE_3_Kd = 0;
        public static final double STAGE_3_Kf = 0.05;
 }

     public class POP {
         public static final double F = 0;
         public static final double R = 0;
         public static final double SPEED = 0.5;
         public static final int FORWARD_PNEUMATIC_CHANNEL = 14;
         public static final int BACKWARD_PNEUMATIC_CHANNEL = 15;
     }

     public class LED {
         public static final double YELLOW = 0.69;
         public static final double PURPLE = 0.91;
         public static final double RED = 0.61;
         public static final double GREEN = 0.77;
         public static final double RAINBOW = -0.99;
     }
}