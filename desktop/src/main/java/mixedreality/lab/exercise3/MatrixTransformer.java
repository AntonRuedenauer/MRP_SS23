package mixedreality.lab.exercise3;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Vector3f;
import com.jme3.math.Vector4f;
import mixedreality.base.mesh.TriangleMesh;

public class MatrixTransformer {

    public Vector4f[] getModelPositions(TriangleMesh triangleMesh, int i) {
        // Set up the model matrix and transform points
        Matrix4f modelMatrix = new Matrix4f().IDENTITY;
        Vector3f position3dA = triangleMesh.getVertex(triangleMesh.getTriangle(i).getA()).getPosition();
        Vector3f position3dB = triangleMesh.getVertex(triangleMesh.getTriangle(i).getB()).getPosition();
        Vector3f position3dC = triangleMesh.getVertex(triangleMesh.getTriangle(i).getC()).getPosition();

        Vector4f position4dA = new Vector4f(position3dA.x, position3dA.y, position3dA.z, 1);
        Vector4f position4dB = new Vector4f(position3dB.x, position3dB.y, position3dB.z, 1);
        Vector4f position4dC = new Vector4f(position3dC.x, position3dC.y, position3dC.z, 1);

        Vector4f pwelt1 = modelMatrix.mult(position4dA);
        Vector4f pwelt2 = modelMatrix.mult(position4dB);
        Vector4f pwelt3 = modelMatrix.mult(position4dC);
        return new Vector4f[]{pwelt1, pwelt2, pwelt3};
    }

    public Vector4f[] getViewPositions(Camera camera, Vector4f[] arr) {
        // Set up the view matrix and transform points
        Matrix4f viewMatrix = getViewMatrix(camera.getEye(), camera.getRef(), camera.getUp());
        Vector4f pcam1 = viewMatrix.mult(arr[0]);
        Vector4f pcam2 = viewMatrix.mult(arr[1]);
        Vector4f pcam3 = viewMatrix.mult(arr[2]);
        return new Vector4f[]{pcam1, pcam2, pcam3};
    }

    public Vector4f[] getProjectionPositions(Vector4f[] arr) {
        // Set up the projection matrix and transform points
        Matrix4f projMatrix = getProjectionMatrix();
        Vector4f pbild1 = projMatrix.mult(arr[0]);
        Vector4f pbild2 = projMatrix.mult(arr[1]);
        Vector4f pbild3 = projMatrix.mult(arr[2]);
        pbild1 = pbild1.divide(pbild1.w);
        pbild2 = pbild2.divide(pbild2.w);
        pbild3 = pbild3.divide(pbild3.w);
        return new Vector4f[]{pbild1, pbild2, pbild3};
    }

    public Vector4f[] getScreenPositions(Vector4f[] arr, Camera camera) {
        // Set up the screenMapping matrix and transform points
        Matrix4f screenMapMatrix = getScreenMapMatrix(camera);
        Vector4f ppixel1 = screenMapMatrix.mult(arr[0]);
        Vector4f ppixel2 = screenMapMatrix.mult(arr[1]);
        Vector4f ppixel3 = screenMapMatrix.mult(arr[2]);
        return new Vector4f[]{ppixel1, ppixel2, ppixel3};
    }

    private Matrix4f getViewMatrix(Vector3f eye, Vector3f ref, Vector3f up) {
        Vector3f tmp = ref.subtract(eye);
        Vector3f z = tmp.divide(tmp.length());
        Vector3f x = up.cross(z);
        Vector3f y = z.cross(x);
        Matrix4f viewMatrix = new Matrix4f(x.x, y.x, z.x, eye.x,
                x.y, y.y, z.y, eye.y,
                x.z, y.z, z.z, eye.z,
                0, 0, 0, 1);
        return viewMatrix;
    }

    private Matrix4f getProjectionMatrix() {
        int z0 = 1;
        Matrix4f projectionMatrix = new Matrix4f(1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 1/z0, 0);
        return projectionMatrix;
    }

    private Matrix4f getScreenMapMatrix(Camera camera) {
        float f = camera.getWidth() / 2 * FastMath.tan(camera.getFovX() / 2);
        Matrix4f screenMapMatrix = new Matrix4f(f, 0, 0, camera.getWidth()/2,
                0, f, 0, camera.getHeight()/2,
                0, 0, 0, 0,
                0, 0, 0, 0);
        return screenMapMatrix;
    }
}
