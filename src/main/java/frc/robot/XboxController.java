package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxController extends CommandXboxController {
    private double curveY = 10;
    private double curveX = 10;
    private double curveZ = 10;

    public XboxController(int port, double curveY, double curveX, double curveZ) {
        super(port);
        this.curveY = curveY;
        this.curveX = curveX;
        this.curveZ = curveZ;
    }

    public XboxController(int port) {
        super(port);
    }

    @Override
    public double getRightY() {
        double linear = super.getRightY();

        if (linear == 0.0) {
            return 0.0;
        }
        double sign = Math.signum(linear);
        double abs = Math.abs(linear);

        return sign * (Math.pow(curveY, abs) - 1.0) / (curveY - 1.0);
    }

    @Override
    public double getRightX() {
        double linear = super.getRightX();

        if (linear == 0.0) {
            return 0.0;
        }
        double sign = Math.signum(linear);
        double abs = Math.abs(linear);

        return sign * (Math.pow(curveX, abs) - 1.0) / (curveX - 1.0);
    }

    @Override
    public double getLeftX() {
        double linear = super.getLeftX();

        if (linear == 0.0) {
            return 0.0;
        }
        double sign = Math.signum(linear);
        double abs = Math.abs(linear);

        return sign * (Math.pow(curveZ, abs) - 1.0) / (curveZ - 1.0);
    }
}
