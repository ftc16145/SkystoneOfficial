package org.firstinspires.ftc.teamcode;

import java.lang.Math;
public class SimpleCalculus {

    public interface CalculusInterface {
        public double function(double x);
    }

    private CalculusInterface calculusInterface;
    public SimpleCalculus(CalculusInterface calculusInterface){
        this.calculusInterface = calculusInterface;
    }

    private double integrate(double from, double to) {
        final int intevalNum = 1000000;
        double inteval = (to - from) / intevalNum;
        double result = 0d;
        for (int i = 0; i < intevalNum; i++) {
            try {
                result += (calculusInterface.function(from + i*inteval) + 
                calculusInterface.function(from + (i + 1)*inteval)) * inteval / 2;
        
            } catch (Exception e) {
                // TODO: handle exception

                System.exit(0);
            }
        }
        return result;
    }

/*private double differentiate(double x){
final double aSmallValue = 0.0000000001;
try {
double d1 = (calculusInterface.function(x + aSmallValue) - 
calculusInterface.function(x))/(aSmallValue);
double d2 = (calculusInterface.function(x - aSmallValue) - 
calculusInterface.function(x))/( - aSmallValue);
double ratio = d1/d2;
if (ratio > 1.01 || ratio < 0.99){
JOptionPane.showMessageDialog(null, "The function is not differntiable at " + x);
System.exit(0);
}
return (d1 + d2) / 2; 
    
} catch (Exception e) {
// TODO: handle exception
JOptionPane.showMessageDialog(null, "The function is not differntiable at " + x);
System.exit(0);
return 0;
}
}*/

public static void main(String[] args) {
    SimpleCalculus calculus = new SimpleCalculus(new CalculusInterface() {
        @Override
        public double function(double x) {
            //the function to be integrated or differentiated; configure it on your own
            return Math.hypot(1, (2*x - 8));
        }
        
    });

    double integrationValue = calculus.integrate(0, 6);
    System.out.println("Integration value is : " + integrationValue);
//double differentiationValue = calculus.differentiate(1);
//System.out.println("Differentiation value is : " + differentiationValue);

    
}

}