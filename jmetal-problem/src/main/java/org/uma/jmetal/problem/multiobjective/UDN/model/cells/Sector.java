/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.uma.jmetal.problem.multiobjective.UDN.model.cells;

import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Pablo Zapata
 */
public class Sector {

    int x_;
    int y_;
    int z_;
    //BTS
    BTS bts_;
    //cells
    List<Cell> cells_;
    UDN udn_;
    int numChainsTX_;
    double transmittedPower_;
    double alfa_;
    double beta_;
    double delta_;
    double transmitterGain_;
    double receptorGain_;
    double coverageRadius_;

    Sector(int x, int y, int z,
           UDN udn,
           BTS bts,
           String antennaType,
           String antennaName,
           int numChainsTX,
           double transmittedPower,
           double alfa,
           double beta,
           double delta,
           double transmitterGain,
           double receptorGain,
           double workingFrequency,
           double coverageRadius) {

        this.x_ = x;
        this.y_ = y;
        this.z_ = z;
        this.udn_ = udn;
        this.bts_ = bts;
        this.numChainsTX_ = numChainsTX;
        this.transmittedPower_ = transmittedPower;
        this.alfa_ = alfa;
        this.beta_ = beta;
        this.delta_ = delta;
        this.transmitterGain_ = transmitterGain;
        this.receptorGain_ = receptorGain;
        this.coverageRadius_ = coverageRadius;
        this.cells_ = new ArrayList<>();


        for (int i = 0; i < numChainsTX_; i++) {
            Cell cell = Cell.newInstance(
                    antennaType,
                    antennaName,
                    this.udn_,
                    this,
                    x,
                    y,
                    z,
                    transmittedPower,
                    alfa,
                    beta,
                    delta,
                    transmitterGain,
                    receptorGain,
                    workingFrequency,
                    this.coverageRadius_
            );
            this.cells_.add(cell);
        }
    }

    public Sector(Sector sector) {
        this.x_ = sector.x_;
        this.y_ = sector.y_;
        this.z_ = sector.z_;
        this.bts_ = sector.bts_;
        this.cells_ = sector.cells_;
        this.udn_ = sector.udn_;
        this.numChainsTX_ = sector.numChainsTX_;
        this.transmittedPower_ = sector.transmittedPower_;
        this.alfa_ = sector.alfa_;
        this.beta_ = sector.beta_;
        this.delta_ = sector.delta_;
        this.transmitterGain_ = sector.transmitterGain_;
        this.receptorGain_ = sector.receptorGain_;
        this.coverageRadius_ = sector.coverageRadius_;
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

    public BTS getBTS() {
        return this.bts_;
    }

    public List<Cell> getCells() {
        return this.cells_;
    }

    public double getTransmittedPower() {
        return transmittedPower_;
    }


    public double getAlfa() {
        return alfa_;
    }

    public double getBeta() {
        return beta_;
    }

    public double getDelta() {
        return delta_;
    }

    public double getTransmitterGain() {
        return transmitterGain_;
    }

    public double getReceptorGain() {
        return receptorGain_;
    }

    public double getCoverageRadius() {
        return coverageRadius_;
    }

    public double getMaximumCoverage() {
        return this.coverageRadius_;
    }

    @Override
    public String toString() {
        return "BTS(" + x_ + "," + y_ + ')';
    }
}
