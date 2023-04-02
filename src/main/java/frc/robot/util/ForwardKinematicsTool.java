package frc.robot.util;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ForwardKinematicsTool {
    public static double[] getArmPosition() {

    //find hypotenuse using law of cosines
    double hypotenuse = Math.sqrt(Math.pow(ArmConstants.kLowerArmLength, 2) + Math.pow(ArmConstants.kUpperArmLength, 2)
            - 2 * ArmConstants.kLowerArmLength * ArmConstants.kUpperArmLength
                    * Math.cos(ArmSubsystem.get().getUpperArmPosition()));

    // find angle d: btwn klowerArmLength and hypotenuse using law of cosines
    double d = Math.acos((Math.pow(ArmConstants.kUpperArmLength, 2) - Math.pow(ArmConstants.kLowerArmLength, 2)
            - Math.pow(hypotenuse, 2)) / (-2 * ArmConstants.kUpperArmLength * hypotenuse));
    
    // find angle f: btwn hypotenuse and x length
    double f = ArmSubsystem.get().getLowerArmPosition() - d;

    double y = hypotenuse * Math.sin(f);
    double x = hypotenuse * Math.cos(f);
    double[] position = {x, y};
    return position;
}
}
