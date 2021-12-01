/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.uma.jmetal.problem.multiobjective.UDN.model.cells;

import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;
import org.uma.jmetal.problem.multiobjective.UDN.model.users.User;

/**
 * @author paco
 */
public abstract class Cell {

    protected static int uniqueId_ = 0;

    //Cell parameters
    int angleShift_;
    int id_;
    UDN.CellType type_;
    String name_;
    UDN udn_;
    BTS bts_;
    Sector sector_;
    boolean active_;
    double cost_;
    double workingFrequency_;
    double wavelength_;
    //MIMO Link Parameters (precomputed)
    double[] singularValuesH; //Singular values of the H matrix
    int numAntennasRx; // # of transmitting antennas
    int numAntennasTx; // # of receiving antennas
    private double totalBW_;
    private int usersAssigned_;

    /**
     * Empty constructor
     */
    public Cell() {
        id_ = uniqueId_;
        bts_ = null;
        sector_ = null;
    }

    public Cell(UDN udn, Sector sector, String cellName, int x, int y, int z, double transmittedPower, double alfa, double beta, double delta, double transmitterGain, double receptorGain, double workingFrequency) {
        this.udn_ = udn;
        this.sector_ = sector;
        this.id_ = uniqueId_;
        uniqueId_++;
        this.name_ = cellName;
        //the bandwidth is 10% of the working frequency
        this.totalBW_ = workingFrequency * 0.1;
        this.usersAssigned_ = 0;

        //Cell configuration
        this.workingFrequency_ = workingFrequency;
        double c = 3e8;
        this.wavelength_ = c / (workingFrequency * 1000000);
    }

    /**
     * Creates a new instance of the class, but in each subclass
     */
    public static Cell newInstance(Cell c) {
        return c.newInstance();
    }

    public static Cell newInstance(String cellType, String cellName, UDN udn, Sector sector, int x, int y, int z, double transmittedPower, double alfa, double beta, double delta, double transmitterGain, double receptorGain, double workingFrequency, double coverageRadius) {

        Cell cell = null;

        if (cellType.equalsIgnoreCase("femto")) {
            cell = new FemtoCell(udn, sector, cellName, x, y, z, transmittedPower, alfa, beta, delta, transmitterGain, receptorGain, workingFrequency, coverageRadius);
        } else if (cellType.equalsIgnoreCase("pico")) {
            cell = new PicoCell(udn, sector, cellName, x, y, z, transmittedPower, alfa, beta, delta, transmitterGain, receptorGain, workingFrequency, coverageRadius);
        } else if (cellType.equalsIgnoreCase("micro")) {
            cell = new MicroCell(udn, sector, cellName, x, y, z, transmittedPower, alfa, beta, delta, transmitterGain, receptorGain, workingFrequency);
        } else if (cellType.equalsIgnoreCase("macro")) {
            cell = new MacroCell(udn, sector, cellName, x, y, z, transmittedPower, alfa, beta, delta, transmitterGain, receptorGain, workingFrequency);
        } else {
            System.out.println("Unknown cell type: " + cellType);
            System.exit(-1);
        }

        return cell;
    }

    abstract Cell newInstance();

    /**
     * Get total demanded capacity
     *
     * @return Traffic demand
     */
    public double getTrafficDemand() {
        int count = 0;
        double totalDemand = 0;
        double totalCapacity = 0;
        double sum = 0;
        int satisfied = 0;
        int unsatisfied = 0;
        double satisfactionRate = 0;

        for (User u : this.udn_.getUsers()) {
            if (u.getServingCell() == this) {
                double userCapacity = u.capacityMIMO(this.udn_, this.getSharedBWForAssignedUsers());
                //double userCapacity = u.capacity(this.udn_, this.getSharedBWForAssignedUsers());
                double userDemand = u.getTrafficDemand() / 1000;

                //If the user demand is satisfied (capacity>demand), the user will "consume" its demand
                //if not satisfied, the upper bound is the capcity of the link
                sum += Math.min(userCapacity, userDemand);
                if (Math.min(userCapacity, userDemand) == userCapacity) {
                    //System.out.println("Unsatisfied");                    
                    unsatisfied++;
                } else {
                    //System.out.println("Satisfied");
                    satisfied++;
                }
                //demand += u.capacityMIMO(this.udn_, this.getSharedBWForAssignedUsers());
                count++;
            }
        }


        if (count != 0) {
            satisfactionRate = (double) satisfied / count;
            //  System.out.println("Satisfaction Rate: " + satisfactionRate + " Users connected: " + this.getAssignedUsers() + " Type: " + this.getType().toString());

        }

        return sum;
    }

    /**
     * Given a pair of angles, this function search the correspondence
     * attenuation factor in a given antenna matrix
     *
     * @param azi  Azi
     * @param occi Occi
     * @return Attenuation factor
     */
    public double getAttenuationFactor(int azi, int occi) {
        if (azi == 0) azi = 1;
        if (occi == 0) occi = 1;

        if (azi + this.angleShift_ > 360)
            azi = azi + this.angleShift_ - 360;
        else
            azi = azi + this.angleShift_;

        //Set vertical tilt depending on the cell type (max factor at 90 deg)
        if (this.type_.equals(UDN.CellType.FEMTO) || this.type_.equals(UDN.CellType.PICO)) {
            occi = occi - 20;
        } else {
            occi = occi - 10;
        }
        //occi=90;
//        return this.getBTS().antennaArray_.getReal(azi - 1, occi - 1).floatValue();
        return this.getBTS().antennaArray_.getFloat(azi - 1, occi - 1);
    }

    public BTS getBTS() {
        return bts_;
    }

    public Sector getSector() {
        return sector_;
    }

    public void setSector(Sector sector) {
        this.sector_ = sector;
        this.bts_ = sector.getBTS();
    }

    public int getID() {
        return this.id_;
    }

    public void setActivation(boolean b) {
        this.active_ = b;
    }

    public boolean isActive() {
        return this.active_;
    }

    public double getCost() {
        return this.cost_;
    }

    public UDN.CellType getType() {
        return this.type_;
    }

    public double getTotalBW() {
        return totalBW_;
    }

    public double getWorkingFrequency() {
        return workingFrequency_;
    }

    public double getWavelength() {
        return wavelength_;
    }

    public int getAngleShift() {
        return this.angleShift_;
    }

    public void setAngleShift(int shift) {
        this.angleShift_ = shift;
    }

    public int getNumAntTx() {
        return this.numAntennasTx;
    }

    public int getNumAntRx() {
        return this.numAntennasRx;
    }

    public double[] getSingularValuesH() {
        return this.singularValuesH;
    }

    public void setSingularValuesH(double[] singularValuesH) {
        this.singularValuesH = singularValuesH;
    }

    public String printType() {
        String type = "";
        switch (this.type_) {
            case MACRO:
                type = "M";
                break;
            case MICRO:
                type = "m";
                break;
            case PICO:
                type = "p";
                break;
            case FEMTO:
                type = "f";
                break;
        }
        return type;
    }

    @Override
    public String toString() {
        return "[" + printType() + "," + bts_.x_ + "," + bts_.y_ + "," + bts_.z_ + "," + angleShift_ + "," + active_ + "," + usersAssigned_ + "]";
    }

    public void addUserAssigned() {
        this.usersAssigned_++;
    }

    public void removeUserAssigned() {
        this.usersAssigned_--;
    }

    public int getAssignedUsers() {
        return this.usersAssigned_;
    }

    public void setNumbersOfUsersAssigned(int v) {
        this.usersAssigned_ = v;
    }

    public double getSharedBWForAssignedUsers() {
        return this.totalBW_ / this.usersAssigned_;
    }
}
