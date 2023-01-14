package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
    DoubleSubscriber xSub;
    DoubleSubscriber ySub;

    public void robotInit() {
        // Get the default instance of NetworkTables that was created automatically
        // when the robot program starts
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // Get the table within that instance that contains the data. There can
        // be as many tables as you like and exist to make it easier to organize
        // your data. In this case, it's a table called datatable.
        NetworkTable table = inst.getTable("datatable");

        // Start publishing topics within that table that correspond to the X and Y values
        // for some operation in your program.
        // The topic names are actually "/datatable/x" and "/datatable/y".
        xSub = table.getDoubleTopic("x").subscribe(0);
        ySub = table.getDoubleTopic("y").subscribe(0);
    }

    double m_x = 0;
    double m_y = 0;

    public void teleopPeriodic() {
        // Publish values that are constantly increasing.
        m_x = xSub.get();
        m_y = ySub.get();
        System.out.println(m_x);
        System.out.println(m_y);
    }
}
