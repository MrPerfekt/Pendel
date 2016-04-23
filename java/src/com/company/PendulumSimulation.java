package com.company;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.*;
import java.util.Timer;

public class PendulumSimulation extends JPanel{
    private int width;
    private int height;
    public double time;

    public PendulumSimulation(int width, int height) {
        super(true);
        this.width = width;
        this.height = height;
        repaint();
    }

    public Dimension getPreferredSize() {
        return new Dimension(width, height);
    }

    public void paintComponent(Graphics graphics) {
        BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g = bufferedImage.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);

        super.paintComponent(g);
        time = ((Calendar.getInstance().getTimeInMillis() / 5) % 1000) / 1000.0;

        g.setColor(Color.BLACK);
        g.fillRect(0, 0, width, height);
        g.setColor(Color.white);
        double sizeX = 10;
        double sizeY = height - sizeX;
        double degree = Math.PI / 5 * Math.sin(time * 2 * Math.PI);
        double pendulumPosX = width / 2 + Math.sin(degree) * sizeY;
        double pendulumPosY = Math.cos(degree) * sizeY;
        int xpoints[] = {width / 2 - 5, width / 2 + 5, (int) (pendulumPosX + 5), (int) (pendulumPosX - 5)};
        int ypoints[] = {0, 0, (int) pendulumPosY, (int) pendulumPosY};
        g.fillPolygon(xpoints, ypoints, 4);

        Graphics2D g2dComponent = (Graphics2D) graphics;
        g2dComponent.drawImage(bufferedImage, null, 0, 0);
    }

    public static void main(String[] args) {
        int width = 1000;
        int height = 1000;
        JFrame frame = new JFrame("Direct draw demo");

        PendulumSimulation pendulum = new PendulumSimulation(width, height);

        frame.add(pendulum);
        frame.pack();
        frame.setVisible(true);
        frame.setResizable(false);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        new Timer().scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                frame.repaint();
            }
        }, 20, 20);

//        double x = 0;
//        while (true) {
//            x += 0.0000002;
//            pendulum.time = ((Calendar.getInstance().getTimeInMillis() / 1) % 1000) / 1000.0;
//            pendulum.repaint();
//        }

    }
}