/**
 * Diese Datei ist Teil des Vorgabeframeworks für die Veranstaltung "Mixed Reality"
 * <p>
 * Prof. Dr. Philipp Jenke, Hochschule für Angewandte Wissenschaften Hamburg.
 */

package mixedreality.lab.exercise3;

import com.jme3.math.FastMath;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import mixedreality.base.mesh.ObjReader;
import mixedreality.base.mesh.TriangleMesh;
import ui.Scene2D;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;

import static mixedreality.base.math.ConvexHull2D.cross;

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

    private MatrixTransformer math = new MatrixTransformer();

    public MyRendererScene(int width, int height) {
        super(width, height);
        camera = new Camera(new Vector3f(0, 0, -2), new Vector3f(0, 0, 0),
                new Vector3f(0, 1, 0), 90.0f / 180.0f * FastMath.PI, 1,
                width, height);
        backfaceCulling = true;
        lastMousePosition = null;

        ObjReader reader = new ObjReader();
        //mesh = reader.read("models/cube.obj");
        mesh = reader.read("Models/deer.obj");

        setupListeners();
    }

    @Override
    public void paint(Graphics g) {
        Graphics2D g2 = (Graphics2D) g;
        g2.setColor(Color.BLUE);

        if (mesh != null) {
            for (int i = 0; i < mesh.getNumberOfTriangles(); i++) {
                Vector3f[] modelMatrix = math.createModelMatrix(mesh, i);
                Vector3f[] viewMatrix = math.createViewMatrix(camera, modelMatrix);
                Vector3f[] projectionMatrix = math.createProjectionMatrix(viewMatrix);
                Vector3f[] screenMappingMatrix = math.createScreenMappingMatrix(projectionMatrix, camera);
                drawTriangle(g2, screenMappingMatrix[0], screenMappingMatrix[1], screenMappingMatrix[2]);
            }
        }
    }

    private void drawTriangle(Graphics2D g2, Vector3f ppixel1, Vector3f ppixel2, Vector3f ppixel3) {
        if (!backfaceCulling) {
            // Draw line
            drawLine(g2, new Vector2f(ppixel1.x, ppixel1.y), new Vector2f(ppixel2.x, ppixel2.y), g2.getColor());
            drawLine(g2, new Vector2f(ppixel1.x, ppixel1.y), new Vector2f(ppixel3.x, ppixel3.y), g2.getColor());
        } else if (triangleIsClockwise(ppixel1, ppixel2, ppixel3)) {
            // Draw line
            drawLine(g2, new Vector2f(ppixel1.x, ppixel1.y), new Vector2f(ppixel2.x, ppixel2.y), g2.getColor());
            drawLine(g2, new Vector2f(ppixel1.x, ppixel1.y), new Vector2f(ppixel3.x, ppixel3.y), g2.getColor());
        }
    }

    private boolean triangleIsClockwise(Vector3f p1, Vector3f p2, Vector3f p3) {
        return cross(new Vector2f(p1.x, p1.y), new Vector2f(p2.x, p2.y), new Vector2f(p3.x, p3.y)) < 0;
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
