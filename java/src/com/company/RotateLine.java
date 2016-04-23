package com.company;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

/**
 * Created by andreas on 12.04.16.
 */
public class RotateLine extends JPanel implements ActionListener {

    private static final int PREF_W = 800;
    private static final int PREF_H = 800;
    private static final int X1 = 100;
    private static final int Y1 = 100;
    private static ArrayList<Point> pointsList;
    private static Point p;
    private int counter = 0;
    private int index = 0;
    Timer time = new Timer(10, (ActionListener) this);

    public RotateLine() {
        pointsList = new ArrayList<Point>();
        p = new Point(X1, Y1);
        int X2 = 400;
        int Y2 = Y1;
        for (int count = 0; count < 300; count++) {
            pointsList.add(new Point(X2, Y2));
            X2 = X2 - 1;
            Y2 = Y2 + 2;
        }
        time.start();
    }

    @Override
    public Dimension getPreferredSize() {
        return new Dimension(PREF_W, PREF_H);
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        drawRotatingLine(g2d);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("clock");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new RotateLine());
        frame.pack();
        frame.setVisible(true);
    }

    public void drawRotatingLine(Graphics2D g) {
        g.drawLine(p.x, p.y, pointsList.get(index).x, pointsList.get(index).y);
    }

    public void actionPerformed(ActionEvent arg0) {
        if (index < pointsList.size() - 1) {
            time.setDelay(20);
            repaint();
            index++;
        }
    }
}