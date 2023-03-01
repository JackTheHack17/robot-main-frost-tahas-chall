package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoard {
    CommandGenericHID leftBoard;
    CommandGenericHID rightBoard;

    public ButtonBoard (int leftBoardPort, int rightBoardPort) {
        leftBoard = new CommandGenericHID(leftBoardPort);
        rightBoard = new CommandGenericHID(rightBoardPort);

        Telemetry.setValue("buttonBoard/0", false);
        Telemetry.setValue("buttonBoard/1", false);
        Telemetry.setValue("buttonBoard/2", false);
        Telemetry.setValue("buttonBoard/3", false);
        Telemetry.setValue("buttonBoard/4", false);
        Telemetry.setValue("buttonBoard/5", false);
        Telemetry.setValue("buttonBoard/6", false);
        Telemetry.setValue("buttonBoard/7", false);
        Telemetry.setValue("buttonBoard/8", false);
        // 9 intentionally skipped
        Telemetry.setValue("buttonBoard/10", false);
        Telemetry.setValue("buttonBoard/11", false);
        Telemetry.setValue("buttonBoard/12", false);
        Telemetry.setValue("buttonBoard/13", false);
        Telemetry.setValue("buttonBoard/14", false);
        Telemetry.setValue("buttonBoard/15", false);
        Telemetry.setValue("buttonBoard/16", false);
    }

    public boolean getLED (int LEDNumber) {
        return Telemetry.getValue("buttonBoard/" + LEDNumber, false);
    }

    public void setLED (int LEDNumber, boolean value) {
        Telemetry.setValue("buttonBoard/" + LEDNumber, value);
    }

    public boolean toggleLED (int LEDNumber) {
        if (getLED(LEDNumber)) {
            setLED(LEDNumber, false);
            return false;
        } else {
            setLED(LEDNumber, true);
            return true;
        }
    }

    public Trigger button ( int buttonNumber ) {
        return (buttonNumber < 9) ? leftBoard.button(buttonNumber) : rightBoard.button(buttonNumber - 9);
    }

    public boolean getRawButton ( int buttonNumber ) {
        return (buttonNumber < 9) ? leftBoard.getHID().getRawButton(buttonNumber) : rightBoard.getHID().getRawButton(buttonNumber - 9);
    }

    private double getAxis ( int axisNumber ) {
        return rightBoard.getRawAxis(axisNumber);
    }

    public Translation2d getJoystick () {
        return new Translation2d(getAxis(0), getAxis(1));
    }
}
