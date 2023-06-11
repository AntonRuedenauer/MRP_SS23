/**
 * Diese Datei ist Teil des Vorgabeframeworks für die Veranstaltung "Mixed Reality"
 * <p>
 * Prof. Dr. Philipp Jenke, Hochschule für Angewandte Wissenschaften Hamburg.
 */

package mixedreality.lab.exercise6;

import com.jme3.math.Vector2f;
import org.apache.commons.lang3.ArrayUtils;
import org.lwjgl.openal.SOFTDeferredUpdates;
import ui.Scene2D;

import java.awt.*;
import java.util.HashMap;
import java.util.Map;

/**
 * Implementation of an L-System
 */
public class LSystemScene2D extends Scene2D {

    private double stepSize;
    private double angle;
    /**
     * The axiom is a single character
     */
    protected String axiom;
    /**
     * All rules are in a map which maps a character to its replacement.
     */
    protected Map<Character, String> rules;
    /**
     * Number of iterations during derivation
     */
    protected int numIterations;

    /**
     * Result of the last derivation.
     */
    protected String currentWord;

    public LSystemScene2D(int width, int height) {
        super(width, height, new Vector2f(-1, -1), new Vector2f(1, 1));
        this.currentWord = "";

        // options
        sceneOne();
        //sceneTwo();

        // Run derivation
        derive();
        // Debugging: show derived word.
        System.out.println(currentWord);
    }

    public void sceneOne() {
        this.axiom = "F+F+F+F";
        this.rules = new HashMap<>();
        this.rules.put('F', "F+F-F");
        this.rules.put('-', "-");
        this.rules.put('+', "+");
        this.stepSize = 0.1;
        this.angle = 90;
        this.numIterations = 4;
    }

    public void sceneTwo() {
        this.axiom = "F-F-F";
        this.rules = new HashMap<>();
        this.rules.put('F', "F-F+F");
        this.rules.put('-', "-");
        this.rules.put('+', "+");
        this.stepSize = 0.1;
        this.angle = 120;
        this.numIterations = 2;
    }

    /**
     * Derive the axiom for the given number of iterations. The result of the
     * derivation must be saved in the variable currentWord.
     */
    public void derive() {
        currentWord = this.axiom;
        for (int i=0; i < numIterations; i++) {
            currentWord = replaceOneRuleset(currentWord);
        }
        if (numIterations > 0 ) {
            stepSize = stepSize  / numIterations;
        }
    }

    public String replaceOneRuleset(String word) {
        StringBuilder localWord = new StringBuilder();
        for (char letter : word.toCharArray()) {
            localWord.append(this.rules.get(letter));
        }
        return localWord.toString();
    }

    @Override
    public void paint(Graphics g) {
        Turtle turtle = new Turtle(0, 0,0);
        String[] wordArray = currentWord.split("");
        for (String element : wordArray) {
            if (element.equals("+")) {
                turtle.changeAngleBy(-angle);
            } else if (element.equals("-")) {
                turtle.changeAngleBy(angle);
            } else {
                Vector2f end = drawLineForTurtle(g, turtle, stepSize);
                turtle.x = end.x;
                turtle.y = end.y;

            }
        }
    }

    private Vector2f drawLineForTurtle(Graphics g, Turtle turtle, double stepSize) {
        Vector2f start = new Vector2f(turtle.x, turtle.y);
        double radiant = Math.toRadians(turtle.angle);
        Vector2f end = new Vector2f((float) (turtle.x + stepSize * Math.cos(radiant)),
                (float) (turtle.y + stepSize * Math.sin(radiant)));
        drawLine(g, start, end, Color.BLACK);
        return end;
    }

    @Override
    public String getTitle() {
        return "L-System";
    }
}
