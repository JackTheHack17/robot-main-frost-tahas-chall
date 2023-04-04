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
        public static final double WHEEL_PERIMETER = WHEEL_DIAMETER * Math.PI;
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
        public static final double SWERVE_SLOW_SPEED_PERCENTAGE = 0.1;
        public static final double ROTATION_SCALE_FACTOR = 0.65;
            
        // pid values
        public static final double AZIMUTH_kP = 0.01; // sds: 0.2; rylan: 0.65
        public static final double AZIMUTH_kD = 0;

        // calculated via JVN calculator
        public static final double DRIVE_kP = 0.044057;
        public static final double DRIVE_kF = 0.028998;

        /* Maximum distance for a valid waypoint (meters) */
        public static final double MAX_WAYPOINT_DISTANCE = 0.5;

        public static final double SHWERVE_DRIVE_Kp = 0.044057;
        public static final double SHWERVE_DRIVE_Kd = 0;

        public static final double AUTO_BALANCE_Kp = 0.1;
        public static final double AUTO_BALANCE_Kd = 0;
    }

    public class LL {
        public static final double SLOPE = 0;
        public static final double YINT = 0;
    }

    public static final class ARM {
        public static final double JOINT_ANGLE_DEADZONE = 20;
        public static final double JOINT_COORDINATE_DEADZONE = 0;

        public static enum positions {
            ScoreHighCone,
            DipHighCone,
            ScoreHighCube,
            ScoreMidCone,
            DipMidCone,
            ScoreMidCube,
            ScoreHighPlace,
            ScoreMidPlace,
            ScoreLow,
            Floor,
            FloorAlt,
            Substation,
            Idle,
            IdleShootPosition
        };

        public static final double STAGE_1_OFFSET = 177;
        public static final double STAGE_2_OFFSET = 270;
        public static final double STAGE_3_OFFSET = 210;

        public static ArmPosition scoreHighConePosition  = new ArmPosition(183, 325, 223);
        public static ArmPosition dipHighConePosition  = new ArmPosition(183, 290, 213);
        public static ArmPosition scoreHighCubePosition  = new ArmPosition(138, 325, 216);
        public static ArmPosition scoreMidConePosition   = new ArmPosition(107, 355, 240);
        public static ArmPosition dipMidConePosition   = new ArmPosition(107, 345, 225);
        public static ArmPosition scoreMidCubePosition   = new ArmPosition(107, 352, 187);
        public static ArmPosition idlePosition   = new ArmPosition(107, 53, 129);
        public static ArmPosition scoreLowPosition      = new ArmPosition(107, 2, 116);
        public static ArmPosition floorPosition      = new ArmPosition(107, 290, 225);
        public static ArmPosition floorAltPosition   = new ArmPosition(107, 319, 120);
        public static ArmPosition substationPosition = new ArmPosition(120, 20, 180);//allows reach all the way back and from edge

        public static final double THETA_SPEED = 1;
        public static final double X_SPEED = 0.5;
        public static final double Y_SPEED = 0.5;

        // inches
        public static final double STAGE_1_LENGTH = 20.0;
        public static final double STAGE_2_LENGTH = 29.5;

        public static final double STAGE_1_Kp = 0.0350;//kp=.008, 0.025
        public static final double STAGE_1_Ki = 0.001;
        public static final double STAGE_1_Kd = 0.000;//0.0005
        public static final double STAGE_1_Ks = 0.0005;
        public static final double STAGE_1_Kg = 0.5;//1.3 causes tiny pulse width modulation
        public static final double STAGE_1_MAX_SPEED = 250;
        public static final double STAGE_1_MAX_ACCEL = 160;
        
        public static final double STAGE_2_Kp = 0.0325;//0.008
        public static final double STAGE_2_Ki = 0.00;//0.0025 //0.0065
        public static final double STAGE_2_Kd = 0.0040;//0.035 0.0043
        public static final double STAGE_2_Ks = 0.05;
        public static final double STAGE_2_Kg = 1.0;//1.4
        public static final double STAGE_2_MAX_SPEED = 250;
        public static final double STAGE_2_MAX_ACCEL = 170;

        public static final double STAGE_3_Kp = 0.0255;//0.005
        public static final double STAGE_3_Ki = 0;
        public static final double STAGE_3_Kd = 0.0005;
        public static final double STAGE_3_Ks = 0.04;
        public static final double STAGE_3_Kg = 0.52;
        public static final double STAGE_3_MAX_SPEED = 500;
        public static final double STAGE_3_MAX_ACCEL = 300;
    }

     public class POP {
         public static final double F = 0;
         public static final double R = 0;
         public static final double SPEEDOUT = 0.2;
         public static final double SPEEDIN = 0.5;

         public static final int FORWARD_PNEUMATIC_CHANNEL = 14;
         public static final int BACKWARD_PNEUMATIC_CHANNEL = 15;
     }

     public class LED {
        //  public static final double YELLOW = 0.69;
        //  public static final double PURPLE = 0.91;
        //  public static final double RED = 0.61;
        //  public static final double GREEN = 0.77;
        //  public static final double RAINBOW = -0.99;
        public final static int YELLOWR = 255;
        public final static int YELLOWG = 255;
        public final static int YELLOWB = 0;
        public final static int PURPLER = 255;
        public final static int PURPLEG = 0;
        public final static int PURPLEB = 255;
        public final static int REDR = 255;
        public final static int REDG = 0;
        public final static int REDB = 0;
        public final static int GREENR = 0;
        public final static int GREENG = 255;
        public final static int GREENB = 0;
     }
}
