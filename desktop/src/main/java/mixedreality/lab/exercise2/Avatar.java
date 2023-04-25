/**
 * Diese Datei ist Teil des Vorgabeframeworks für die Veranstaltung "Mixed Reality"
 * <p>
 * Prof. Dr. Philipp Jenke, Hochschule für Angewandte Wissenschaften Hamburg.
 */

package mixedreality.lab.exercise2;

import com.jme3.math.Matrix3f;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import math.Vectors;

/**
 * Information about the avatar in the game.
 */
public class Avatar {

    /**
     * Change rate of the orientation per time step.
     */
    public static final float ROTATION_VELOCITY = 0.1f;
    /**
     * Change rate of the position per time step.
     */
    public static final float MOVE_VELOCITY = 0.01f;
    /**
     * Current position of the avatar
     */
    protected Vector2f pos;
    /**
     * Current rotation angle of the avatar (radian measure)
     */
    protected float rotationAngle;
    /**
     * The avatar must be rotated that its orientation matches this vector.
     */
    protected Vector2f targetPos;
    float MAX_ANGLE = (float) Math.PI / 2;

    public Avatar() {
        pos = new Vector2f(0, 0);
        rotationAngle = 0;
        targetPos = null;
    }

    public Vector2f getPos() {
        // Pos = translation part of pose matrix
        return Vectors.xy(makePose().mult(Vector3f.UNIT_Z));
    }

    public Vector2f getOrientation() {
        // Orientation of first colum of rotation part of pose matrix.
        return Vectors.xy(makePose().mult(Vector3f.UNIT_X));
    }

    public void setTargetPos(Vector2f o) {
        this.targetPos = o;
    }

    // ++++++++++++++++ YOUR TASKS START HERE +++++++++++++++++++++++++++++++++

    /**
     * Generate a 3x3 homogenious transformation matrix which contains the
     * current rotation and p
     */
    protected Matrix3f makePose() {
        float cos = (float) Math.cos(rotationAngle);
        float sin = (float) Math.sin(rotationAngle);
        return new Matrix3f(cos, -sin, pos.x,
                sin, cos, pos.y,
                0, 0, 1);
    }

    /**
     * Move the avatar along the current orientation.
     */
    public void moveToTargetPos() {
        // Check if there is a target position
        if (targetPos != null) {
            Vector3f targetVector = new Vector3f(targetPos.x - getPos().x, targetPos.y - getPos().y, 1);
            float angle = getOrientation().angleBetween(Vectors.xy(targetVector));

            if (angle < MAX_ANGLE) {
                moveAStep(targetVector, angle);
                checkIfReachedTarget();
            } else {
                turn(angle);
            }
        }
    }

    private void turn(float angle) {
        System.out.println(angle);
        if (angle < -ROTATION_VELOCITY) {
            rotationAngle -= ROTATION_VELOCITY;
        } else if (angle > ROTATION_VELOCITY) {
            rotationAngle += ROTATION_VELOCITY;
        } else {
            rotationAngle += angle;
        }
    }

    private void checkIfReachedTarget() {
        // Stop avatar: Compute distance to target
        float distanceToTarget = pos.distance(targetPos);
        if (distanceToTarget <= 2 * MOVE_VELOCITY) {
            targetPos = null; // stop moving
        }
    }

    private void moveAStep(Vector3f targetPosition, float angle) {
        turn(angle);
        // Update position
        pos = pos.add(Vectors.xy(targetPosition).multLocal(MOVE_VELOCITY));
    }
}
