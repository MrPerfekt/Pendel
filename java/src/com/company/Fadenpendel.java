package com.company;
import java.awt.*;
import javax.swing.*;

public class Fadenpendel extends JApplet implements Runnable {

//-----------------------------------------------------------------------------
// Konstanten
//-----------------------------------------------------------------------------


    final int R = 20;                         // Radius Pendelkörper (Pixel)
    final int R2 = 2*R;                       // Durchmesser Pendelkörper (Pixel)
    final int L = 200;                        // Pendellänge (Pixel)
    final double A = 0.4;                     // Amplitude (Bogenmaß);
    // entspricht etwa 23°
    final double T = 2;                       // Schwingungsdauer (s)
    final double OMEGA = 2*Math.PI/T;         // Kreisfrequenz (1/s)
    final Color FZF = Color.yellow;           // Hintergrundfarbe Zeichenfläche


//-----------------------------------------------------------------------------
// Attribute
//-----------------------------------------------------------------------------


    Container cp;                             // Container für Komponenten
    int breite, höhe;                         // Abmessungen (Pixel)
    Zeichenfläche zf;                         // Zeichenfläche
    int ax, ay;                               // Aufhängung des Pendels (Pixel)
    double phi;                               // Phase (Bogenmaß)
    Thread thr;                               // Prozess für Animation
    double t;                                 // Zeit (s)


//-----------------------------------------------------------------------------
// Weitere Methoden
//-----------------------------------------------------------------------------

    // Start-Methode:

    public void start () {
        cp = getContentPane();                  // Container für Komponenten
        cp.setLayout(null);                     // Hartes Layout (Pixelangaben)
        Dimension dim = getSize();              // Abmessungen (aus HTML-Datei)
        breite = dim.width;                     // Breite (Pixel)
        höhe = dim.height;                      // Höhe (Pixel)
        ax = breite/2;                          // x-Koordinate Aufhängung (Pixel)
        ay = 40;                                // y-Koordinate Aufhängung (Pixel)
        zf = new Zeichenfläche();               // Zeichenfläche (innere Klasse)
        zf.setBackground(FZF);                  // Hintergrundfarbe Zeichenfläche
        zf.setBounds(0,0,breite,höhe);          // Position/Abmessungen Zeichenfläche
        cp.add(zf);                             // Zeichenfläche zum Container hinzufügen
        thr = new Thread(this);                 // Prozess für Animation
        thr.start();                            // Prozess starten
    }

    // Stopp-Methode:

    public void stop () {
        thr = null;                             // Prozess für Garbage Collection freigeben
        cp.removeAll();                         // Komponenten entfernen
    }

    // Methode für Animation
    // (Schleife über eine Schwingung):

    public void run () {
        long t0, t1;
        t0 = System.currentTimeMillis();        // Zeit in ms
        while (thr == Thread.currentThread()) { // Solange Prozess thr aktiv ...
            if (t <= T)                           // ... Falls erste Periode ...
                zf.repaint();                       // ... Zeichenfläche neu zeichnen
            try {Thread.sleep(50);}               // ... 50 ms Unterbrechung
            catch (InterruptedException ie) {}
            t1 = System.currentTimeMillis();      // ... Zeit in ms
            t += (t1-t0)/1000.0;                  // ... Zeit-Variable aktualisieren (in s)
            t0 = t1;                              // ... Zeit in ms
        }
    }

    // Pendel zeichnen:
    // g ... Grafik-Kontext

    void pendel (Graphics g) {
        // Zur Zeit t = 0 ist das Pendel
        // nach links maximal ausgelenkt.
        phi = A*Math.cos(OMEGA*t);		   // Phase (Bogenmaß)
        // Im Gradmaß Auslenkung
        // etwa von 23° bis -23°

        int px, py;                             // Mittelpunktskoordinaten Pendelkörper
        px = (int)Math.round(ax-L*Math.sin(phi));  // x-Koordinate Kugelmittelpunkt
        py = (int)Math.round(ay+L*Math.cos(phi));  // y-Koordinate Kugelmittelpunkt
        g.setColor(Color.black);                // Faden
        g.drawLine(ax,ay,px,py);
        g.setColor(Color.blue);                 // Pendelkörper (Kreisfläche)
        g.fillOval(px-R,py-R,R2,R2);
        g.setColor(Color.black);                // Pendelkörper (Kreisrand)
        g.drawOval(px-R,py-R,R2,R2);
    }


    // --------------------------------------------------------------------------


    // Innere Klasse Zeichenfläche:


    class Zeichenfläche extends JPanel {


        // Grafik-Ausgabe:
        // g ... Grafik-Kontext


        public void paint (Graphics g) {
            super.paint(g);                       // Hintergrund zeichnen
            g.setColor(Color.black);              // Aufhängung zeichnen
            g.fillRect(40,ay-4,breite-80,4);
            pendel(g); 					   // Pendel zeichnen
        }


    } // Ende Zeichenfläche

} // Ende Fadenpendel
