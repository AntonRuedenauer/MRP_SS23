/**
 * Diese Datei ist Teil des Vorgabeframeworks für die Veranstaltung "Mixed Reality"
 * <p>
 * Prof. Dr. Philipp Jenke, Hochschule für Angewandte Wissenschaften Hamburg.
 */

package mixedreality.lab.exercise2;

import com.jme3.math.Matrix3f;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import sprites.AnimatedSprite;
import sprites.Constants;
import sprites.SpriteAnimationImporter;
import ui.Scene2D;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.Timer;
import java.util.TimerTask;

import static math.Matrices.transform;
import static sprites.Constants.WALK_ANIMATION_IDS;

/**
 * Base scene for assigment 2
 */
public class Assignment2Scene2D extends Scene2D implements MouseListener {
    private final Vector2f arrowPos;

    //private final Sprite staticSprite;
    /**
     * This class controls the behavior of the avatar
     */
    protected Avatar avatar;
    /**
     * Representation of the animated sprite used for the avatar
     */
    private AnimatedSprite avatarSprite;
    /**
     * The current position of the mouse cursor in scene coordinates.
     */
    private Vector2f mousePosInScene;

    public Assignment2Scene2D(int width, int height) {
        super(width, height, new Vector2f(-1, -1), new Vector2f(1, 1));
        avatar = new Avatar();
        this.mousePosInScene = new Vector2f(-1, -1);

        // Avatar
        loadAvatarSprite();
        avatarSprite.setAnimationId(Constants.WalkAnimations.WALK_E);

        // Arrow
        // this.staticSprite = new Sprite("sprites/staticSprite.png", 50, 50);
        this.arrowPos = new Vector2f(-0.5f, -0.5f);

        // This is the game loop
        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                SwingUtilities.invokeLater(() -> {
                    updateGame();
                    renderGame();
                });
            }
        }, 0, 100);
        addMouseListener(this);
    }

    /**
     * This method is called once before rendering and is used to update the game state.
     */
    private void updateGame() {
        // Move avatar into current orientation
        avatar.moveToTargetPos();
    }

    /**
     * This method is called one per render frame.
     */
    private void renderGame() {
        avatarSprite.setAnimationId(computeAnimationForOrientation());
        repaint();
    }

    @Override
    public void paint(Graphics g) {
        g.clearRect(0, 0, getWidth(), getHeight());
        // Draw target
        if (mousePosInScene != null) {
            drawPoint(g, mousePosInScene, Color.BLACK);
        }

        // Show orientation
        drawLine(g, avatar.getPos(),
                avatar.getPos().add(avatar.getOrientation().mult(0.2f)), Color.LIGHT_GRAY);

        // Draw avatar sprite
        avatarSprite.draw(g, world2Pixel(avatar.getPos()));

        // Draw arrow
        Matrix3f pose = getArrowPose(avatar, arrowPos);
        Vector2f start = new Vector2f(0, 0);
        Vector2f end = new Vector2f(0.2f, 0);
        Vector2f s = transform(pose, start);
        Vector2f e = transform(pose, end);
        drawPoint(g, s, Color.BLUE);
        drawLine(g, s, e, Color.BLUE);
    }

    @Override
    public String getTitle() {
        return "Assignment 2 - Game";
    }

    /**
     * Pre-load of sprite animations, generate animated sprite object.
     */
    private void loadAvatarSprite() {
        SpriteAnimationImporter.ImportParams[] importParams = {
                new SpriteAnimationImporter.ImportParams(
                        "sprites/character_sprites.png",
                        WALK_ANIMATION_IDS[Constants.WalkAnimations.WALK_S.ordinal()],
                        64, 64,
                        new SpriteAnimationImporter.Idx(0, 10),
                        SpriteAnimationImporter.Orientation.HORIZONTAL,
                        9, false
                ),
                new SpriteAnimationImporter.ImportParams(
                        "sprites/character_sprites.png",
                        WALK_ANIMATION_IDS[Constants.WalkAnimations.WALK_W.ordinal()],
                        64, 64,
                        new SpriteAnimationImporter.Idx(0, 9),
                        SpriteAnimationImporter.Orientation.HORIZONTAL,
                        9, false
                ),
                new SpriteAnimationImporter.ImportParams(
                        "sprites/character_sprites.png",
                        WALK_ANIMATION_IDS[Constants.WalkAnimations.WALK_N.ordinal()],
                        64, 64,
                        new SpriteAnimationImporter.Idx(0, 8),
                        SpriteAnimationImporter.Orientation.HORIZONTAL,
                        9, false
                ),
                new SpriteAnimationImporter.ImportParams(
                        "sprites/character_sprites.png",
                        WALK_ANIMATION_IDS[Constants.WalkAnimations.WALK_E.ordinal()],
                        64, 64,
                        new SpriteAnimationImporter.Idx(0, 11),
                        SpriteAnimationImporter.Orientation.HORIZONTAL,
                        9, false
                )
        };
        avatarSprite = SpriteAnimationImporter.importAnimatedSprite(importParams);
    }

    @Override
    public void mouseClicked(MouseEvent e) {
        mousePosInScene = pixel2World(new Vector2f(e.getX(), e.getY()));
        avatar.setTargetPos(mousePosInScene);
    }

    @Override
    public void mousePressed(MouseEvent e) {
    }

    @Override
    public void mouseReleased(MouseEvent e) {
    }

    @Override
    public void mouseEntered(MouseEvent e) {
    }

    @Override
    public void mouseExited(MouseEvent e) {
    }

    // +++++++++ YOUR TASKS START HERE +++++++++++++++++++++++++++++++++++++++++

    /**
     * Compute the walking animation constant for the current avatar rotation.
     */
    protected Constants.WalkAnimations computeAnimationForOrientation() {
        Vector2f orientation = avatar.getOrientation();

        int xRound = Math.round(orientation.x);
        int yRound = Math.round(orientation.y);

        // Avatar zeigt nach Osten
        if (xRound == 1 && yRound == 0) {
            return Constants.WalkAnimations.WALK_E;
        }
        // Avatar zeigt nach Westen
        else if (xRound == -1 && yRound == 0) {
            return Constants.WalkAnimations.WALK_W;
        }
        // Avatar zeigt nach Norden
        else if ((xRound == 0 && yRound == 1) || (xRound == -1 && yRound == 1)) {
            return Constants.WalkAnimations.WALK_N;
        }
        // Avatar zeigt nach Süden
        else if ((xRound == 0 && yRound == -1) || (xRound == -1 && yRound == -1)) {
            return Constants.WalkAnimations.WALK_S;
        }
        // Default
        else {
            return Constants.WalkAnimations.WALK_E;
        }
    }

    /**
     * Return a 2D rotation matrix for the static sprite. The sprite's orientation
     * should follow the avatar.
     */
    protected Matrix3f getArrowPose(Avatar avatar, Vector2f spritePos) {
        if (arrowPos == null) {
            return new Matrix3f();
        }

        // First basis vector
        Vector2f directionToAvatar = avatar.pos.subtract(arrowPos).normalize();
        Vector3f directionAvatar = new Vector3f(directionToAvatar.x, directionToAvatar.y, 0);
        // Second basis vector
        Vector3f secondBasisVector = new Vector3f(directionToAvatar.y, -directionToAvatar.x, 0);
        // Translation vector to arrow position
        Vector3f translationToArrow = new Vector3f(arrowPos.x, arrowPos.y, 1);

        // Combine the rotation and translation vectors into a single pose matrix
        Matrix3f transformationMatrix = new Matrix3f();
        transformationMatrix.setColumn(0, directionAvatar);
        transformationMatrix.setColumn(1, secondBasisVector);
        transformationMatrix.setColumn(2, translationToArrow);

        return transformationMatrix;
    }
}