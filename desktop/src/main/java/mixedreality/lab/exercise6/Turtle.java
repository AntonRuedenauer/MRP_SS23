package mixedreality.lab.exercise6;

public class Turtle {
    double angle;
    float x;
    float y;

    public Turtle(float angle, int x, int y) {
        this.angle = angle;
        this.x = x;
        this.y = y;
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public  void  changeAngleBy(double degree) {
        this.angle = angle + degree;
    }

    public float getX() {
        return x;
    }

    public void setX(float x) {
        this.x = x;
    }

    public float getY() {
        return y;
    }

    public void setY(float y) {
        this.y = y;
    }
}
