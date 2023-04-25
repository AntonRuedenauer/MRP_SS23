package mixedreality.base.math;

public class Mathematics {
    public static int getFactorial(int f) { // Berechnen der Fakultät mit BigInteger (Java Fakultät math)
        int result = 1;
        int i = 0;
        while (i < f) {
            result = result * (f-i);
            i ++;
        }
        return result;
    }

    public static double pow(float base, int exponent) {
        double result = Math.pow(base, exponent);
        if (Double.isNaN(result) || Double.isInfinite(result)) return 1;
        return result;
    }
}