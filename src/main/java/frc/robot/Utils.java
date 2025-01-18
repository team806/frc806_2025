package frc.robot;

public class Utils {
    public static boolean IsDoubleApproximately(double left, double right, double sigma){
        return left >= right - sigma && left <= right + sigma;
    }
}
