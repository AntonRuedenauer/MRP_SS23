package mixedreality.lab.exercise1;

import mixedreality.base.math.BasisFunction;
import mixedreality.base.math.Mathematics;

public class BezierBasisfunktion implements BasisFunction {

    @Override
    public float eval(float t, int index, int degree) {
        return getBinomialkoeffizient(index, degree) *
                (float) Mathematics.pow(t, index) *
                (float) Mathematics.pow(1 - t, degree - index);
    }

    @Override
    public float evalDerivative(float t, int i, int degree) {
        int x = getBinomialkoeffizient(i, degree);
        int n = degree;
        return i * x * (float) Mathematics.pow(1 - t, n - i) * (float) Mathematics.pow(t, i - 1) -
                (n - i) * x * (float) Mathematics.pow(1 - t, n - i - 1) * (float) Mathematics.pow(t, i);
    }

    private int getBinomialkoeffizient(int index, int degree) {
        return Mathematics.getFactorial(degree) /
                (Mathematics.getFactorial(index) * Mathematics.getFactorial(degree - index));
    }
}
