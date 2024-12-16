package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase
{

    public static final String LIMELIGHT = "limelight";
    public static final double SHOOTER_POSITION = 1;
    // adjustable transform for the limelight pose per-alliance 
    private static final Transform2d LL_BLUE_TRANSFORM = new Transform2d(0, 0, new Rotation2d());
    







    /**Returns info about the april tag (x pos, y pos, screen area, and id)*/
    public double[] limelight()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-one");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tid = table.getEntry("tid");

        //read values periodcally
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double Area = ta.getDouble(0.0);
        double Tid = tid.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", Area);
        SmartDashboard.putNumber("LimelightTid", Tid);

        //creates an array to return
        double[] ret = new double[]{x, y, Area, Tid};
        return ret;
    }
}
