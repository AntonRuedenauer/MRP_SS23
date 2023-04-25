/**
 * Diese Datei ist Teil des Vorgabeframeworks für die Veranstaltung "Mixed Reality"
 * <p>
 * Prof. Dr. Philipp Jenke, Hochschule für Angewandte Wissenschaften Hamburg.
 */

package mixedreality.lab.exercise3;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Matrix4f;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.math.Vector4f;
import mixedreality.base.mesh.ObjReader;
import mixedreality.base.mesh.Triangle;
import mixedreality.base.mesh.TriangleMesh;
import ui.Scene2D;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;

/**
 * Drawing canvas for a 3D renderer.
 */
public class MyRendererScene extends Scene2D {

  /**
   * This mesh is rendered
   */
  protected TriangleMesh mesh;

  /**
   * Virtual camera.
   */
  protected Camera camera;

  /**
   * This flag enables/disables backface culling
   */
  protected boolean backfaceCulling;

  public MyRendererScene(int width, int height) {
    super(width, height);
    camera = new Camera(new Vector3f(0, 0, -2), new Vector3f(0, 0, 0),
            new Vector3f(0, 1, 0), 90.0f / 180.0f * FastMath.PI, 1,
            width, height);
    backfaceCulling = true;
    lastMousePosition = null;

    ObjReader reader = new ObjReader();
    mesh = reader.read("models/cube.obj");
    //mesh = reader.read("Models/deer.obj");

    setupListeners();
  }

  @Override
  public void paint(Graphics g) {
    Graphics2D g2 = (Graphics2D) g;

    if (mesh != null) {
      for(int i = 0; i < mesh.getNumberOfTriangles(); i++) {

        // Set up the model matrix
        Matrix4f modelMatrix = new Matrix4f().IDENTITY;
        // create pwelt -> pwelt = modelMatrix.mult(positionOfVerticeInTriangle)
        // TODO: Iterate through every triangle and vertice and transform its position with the matrices and draw a line between them

        // Set up the view matrix
        Matrix4f viewMatrix = getViewMatrix(camera.getEye(), camera.getRef(), camera.getUp());

        // create pcam -> pcam = viewMatrix.mult(p)

        // Set up the projection matrix
        Matrix4f projMatrix = getProjectionMatrix();
        // create pbild -> pbild = projMatrix.mult(pcam)
        // dont forget to divide w

        // Set up the screenmapping matrix
        Matrix4f screenMapMatrix = getScreenMapMatrix();

        // create ppixel -> ppixel = screenMapMatrix.mult(pbild)
      }
    }
  }

  public Matrix4f getViewMatrix(Vector3f eye, Vector3f ref, Vector3f up) {
    Vector3f tmp = eye.subtract(ref);
    Vector3f z = tmp.divide(tmp.length());
    Vector3f x = up.cross(z);
    Vector3f y = z.cross(x);
    Matrix4f viewMatrix = new Matrix4f(x.x, y.x, z.x, eye.x,
            x.y, y.y, z.y, eye.y,
            x.z, y.z, z.z, eye.z,
            0, 0, 0, 1);
    return viewMatrix;
  }

  public Matrix4f getProjectionMatrix() {
    int z0 = 1;
    Matrix4f projectionMatrix = new Matrix4f(1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 1/z0, 0);
    return projectionMatrix;
  }

  public Matrix4f getScreenMapMatrix() {
    float f = camera.getWidth() / 2 * FastMath.tan(camera.getFovX() / 2);
    Matrix4f screenMapMatrix = new Matrix4f(f, 0, 0, camera.getWidth()/2,
            0, f, 0, camera.getHeight()/2,
            0, 0, 0, 0,
            0, 0, 0, 0);
    return screenMapMatrix;
  }

  @Override
  public String getTitle() {
    return "2D Renderer";
  }

  /**
   * Draw a line using the given coordinates (no further transformations).
   */
  public void drawLine(Graphics2D gc, Vector2f a, Vector2f b, Color color) {
    gc.setColor(color);
    gc.drawLine((int) a.x, (int) a.y, (int) b.x, (int) b.y);
  }

  @Override
  public JPanel getUserInterface() {
    JPanel panel = new JPanel();
    panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));

    JCheckBox cbBackfaceCulling = new JCheckBox("backfaceCulling");
    cbBackfaceCulling.setSelected(backfaceCulling);
    cbBackfaceCulling.addActionListener(e -> {
      backfaceCulling = cbBackfaceCulling.isSelected();
      repaint();
    });
    panel.add(cbBackfaceCulling);

    return panel;
  }

  /**
   * Setup listeners - used for user interaction.
   */
  public void setupListeners() {
    addMouseMotionListener(new MouseMotionListener() {
      @Override
      public void mouseDragged(MouseEvent e) {
        Vector2f mPos = new Vector2f(e.getX(), e.getY());
        if (lastMousePosition != null) {
          float dx = mPos.x - lastMousePosition.x;
          float dy = mPos.y - lastMousePosition.y;
          camera.rotateHorizontal(dx);
          camera.rotateVertical(dy);
          repaint();
        }
        lastMousePosition = mPos;
      }

      @Override
      public void mouseMoved(MouseEvent e) {
      }
    });

    addMouseListener(new MouseAdapter() {
      @Override
      public void mouseClicked(MouseEvent e) {
        lastMousePosition = null;
      }

      @Override
      public void mouseReleased(MouseEvent e) {
        lastMousePosition = null;
      }
    });
  }
}
