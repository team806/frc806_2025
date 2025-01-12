package frc.robot;

import frc.robot.Constants;
public class Utils {
    public static boolean IsDoubleApproximately(double left, double right){
        return left >= right - Constants.Delta && left <= right + Constants.Delta;
    }
}
