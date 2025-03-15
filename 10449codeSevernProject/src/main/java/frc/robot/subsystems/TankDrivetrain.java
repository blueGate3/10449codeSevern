package frc.robot.subsystems;

import frc.robot.subsystems.SwerveModule;

public class TankDrivetrain {

    private SwerveModule m_frontRight;
    private SwerveModule m_frontLeft;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;

    public TankDrivetrain() {
        m_frontRight = new SwerveModule(1, 2, false);
        m_frontLeft = new SwerveModule(3, 4, false);
        m_backRight = new SwerveModule(5, 6, false);
        m_backLeft = new SwerveModule(7, 8, false);
    }

    /**
     * a really, *really* bad way to run tank drive. but yk, we ball. 
     * @param leftStick left stick value. only apply left stick 
     * @param rightStick
     */
    public void drive (double leftStick, double rightStick) {
        //controls left half
        m_frontLeft.setDrivePower(leftStick);
        m_backLeft.setDrivePower(leftStick);

        //will need to reset zero positions in rev hardware client
        m_frontLeft.setTurnPosition(0);
        m_backLeft.setTurnPosition(0);

        //same process for right side
        m_frontRight.setDrivePower(rightStick);
        m_backRight.setDrivePower(rightStick);
        m_frontRight.setTurnPosition(0);
        m_backRight.setTurnPosition(0);
    }
}
