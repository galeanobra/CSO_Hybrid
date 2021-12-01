/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.uma.jmetal.problem.multiobjective.UDN.model;

import org.uma.jmetal.problem.multiobjective.UDN.model.cells.BTS;

/**
 * @author paco
 */
public class SocialAttractor {

    private final int id_;
    private int x_;
    private int y_;
    private int z_;

    /**
     * Parametrized constructor
     *
     * @param x_ Coordinate x of the region attraction point
     * @param y_ Coordinate y of the region attraction point
     */
    public SocialAttractor(int id, int x_, int y_, int z_) {
        this.id_ = id;
        this.x_ = x_;
        this.y_ = y_;
        this.z_ = z_;
    }

    public int getX() {
        return x_;
    }

    public int getY() {
        return y_;
    }

    public int getZ() {
        return z_;
    }


//TODO Not adapted to 3D

    void moveSATowardsBTS(BTS b, double alpha) {
        double x1 = this.x_;
        double y1 = this.y_;
        double z1 = this.z_;
        double x2 = b.getX();
        double y2 = b.getY();
        double z2 = b.getZ();

        this.x_ = (int) (alpha * x2 + (1 - alpha) * x1);
        this.y_ = (int) (alpha * y2 + (1 - alpha) * y1);
        this.z_ = (int) (alpha * z2 + (1 - alpha) * z1);

        //this.y_ = 

    }

    @Override
    public String toString() {
        return "SA(" + x_ + "," + y_ + "," + z_ + ')';
    }
}
