package org.uma.jmetal.problem.multiobjective.UDN.model.users;

import org.uma.jmetal.problem.multiobjective.UDN.model.Point;
import org.uma.jmetal.problem.multiobjective.UDN.model.SocialAttractor;
import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Cell;
import org.uma.jmetal.util.PPP;

import java.util.Random;

/**
 * @author paco
 */
public class User {

    //id
    int id_;
    //current position
    int x_;
    int y_;
    int z_;
    //initial position
    int x0_;
    int y0_;
    int z0_;
    //target position
    int x_t;
    int y_t;
    int z_t;

    //velocity for dynamic
    double[] velocity;

    //number of antennas depending on each technology
    int numFemtoAntennas_;
    int numPicoAntennas_;
    int numMicroAntennas_;
    int numMacroAntennas_;

    //traffic demand
    double trafficDemand_;
    double normalDemand_ = 500;
    double heavyDemand_ = 2000;
    double residualTraffic_ = 10;

    //activity
    boolean active_;
    //typename
    String typename_;
    //PPP for demand distribution
    Random random_;
    PPP ppp_;
    double lambdaForPPP_ = 1000;
    //Assignment info
    private Cell servingCell_;

    public User(int id,
                int x, int y, int z,
                double demand,
                String typename,
                boolean isActive,
                int numFemtoAntennas,
                int numPicoAntennas,
                int numMicroAntennas,
                int numMacroAntennas) {
        this.id_ = id;
        this.x_ = x;
        this.y_ = y;
        this.z_ = z;
        this.active_ = isActive;
        if (isActive) {
            this.trafficDemand_ = demand;
        } else {
            this.trafficDemand_ = this.residualTraffic_;
        }
        this.typename_ = typename;
        this.numFemtoAntennas_ = numFemtoAntennas;
        this.numPicoAntennas_ = numPicoAntennas;
        this.numMicroAntennas_ = numMicroAntennas;
        this.numMacroAntennas_ = numMacroAntennas;
        this.servingCell_ = null;

        this.random_ = new Random();
        this.ppp_ = new PPP(this.random_);
    }

    @Override
    public String toString() {
        return "U(" + x_ + "," + y_ + ")";
    }

    public int getX() {
        return x_;
    }

    public void setX(int x) {
        this.x_ = x;
    }

    public int getY() {
        return y_;
    }

    public void setY(int y) {
        this.y_ = y;
    }

    public int getZ() {
        return z_;
    }

    public void setZ(int z) {
        this.z_ = z;
    }

    public int getX_t() {
        return x_t;
    }

    public void setX_t(int x_t) {
        this.x_t = x_t;
    }

    public int getY_t() {
        return y_t;
    }

    public void setY_t(int y_t) {
        this.y_t = y_t;
    }

    public int getZ_t() {
        return z_t;
    }

    public void setZ_t(int z_t) {
        this.z_t = z_t;
    }

    public int getID() {
        return id_;
    }

    public String getUserTypename() {
        return this.typename_;
    }

    public double[] getVelocity() {
        return velocity;
    }

    public void setVelocity(double[] velocity) {
        this.velocity = velocity;
    }

    public void moveUserTowardsSA(SocialAttractor sa, UDN udn, double meanBeta) {
        double x1 = this.x_;
        double y1 = this.y_;
        double z1 = this.z_;
        double x2 = sa.getX();
        double y2 = sa.getY();
        double z2 = sa.getZ();

        //draw beta randomly
        double sigma = (0.5 - Math.abs(meanBeta - 0.5)) / 3.0;
        double beta = meanBeta + udn.getRandom().nextGaussian() * sigma;

        this.x_ = (int) (beta * x2 + (1 - beta) * x1);
        this.y_ = (int) (beta * y2 + (1 - beta) * y1);
        this.z_ = (int) (beta * z2 + (1 - beta) * z1);
    }

    public double getTrafficDemand() {
        double demand;
        if (this.active_) {
            demand = trafficDemand_;
        } else {
            demand = this.residualTraffic_;
        }
        return demand;
    }

    public double userRequiredBWAtCell(UDN udn, Cell c) {
        double sinr, log2sinr, userBW;
        Point p = udn.getGridPoint(x_, y_, z_);

        //sinr in dB
        sinr = p.computeSINR(c);

        log2sinr = Math.log1p(sinr) / Math.log(2.0);

        userBW = this.trafficDemand_ / log2sinr;

        return userBW;
    }

    public double capacity(UDN udn, double bw) {
        Point p = udn.getGridPoint(x_, y_, z_);

        double sinr = p.computeSINR(this.servingCell_);
        double log2sinr = Math.log1p(sinr) / Math.log(2.0);

        return bw * log2sinr;   // Capacity
    }

    /**
     * Retrieves the capacity received by the user considering a MIMO system
     * From "Performance Limits of Multiple-Input Multiple-Output Wireless Communication Systems"
     *
     * @param udn
     * @param bw
     * @return
     */
    public double capacityMIMO(UDN udn, double bw) {
        //We assume that the transmit power of each antenna is P/nt
        Point p = udn.getGridPoint(x_, y_, z_);
        Cell servingCell = this.getServingCell();

        double sinr = p.computeSINR(servingCell);
        int nt = servingCell.getNumAntTx();
        double capacity = 0;
        double[] singularValues = servingCell.getSingularValuesH();

        for (double singularValue : singularValues) {
            capacity += Math.log1p((sinr * singularValue) / nt) / Math.log(2.0);
        }

        return bw * capacity;
    }

    public Cell getServingCell() {
        return this.servingCell_;
    }

    public void setServingCell(Cell c) {
        this.servingCell_ = c;
    }

    public boolean isActive() {
        return this.active_;
    }

    public void activateUser() {
        this.active_ = true;
        if (this.typename_.equals("heavy")) {
            this.trafficDemand_ = this.heavyDemand_;
        } else if (this.typename_.equals("normal")) {
            this.trafficDemand_ = this.normalDemand_;
        }

    }

    public void deactivateUser() {
        this.active_ = false;
        this.trafficDemand_ = this.residualTraffic_;
    }

    public void updateDemand() {
        //TODO: use specific user associated demand for the density of the PPP
        this.trafficDemand_ = ppp_.getPoisson(this.trafficDemand_);
        // System.out.println("new demand: "+this.trafficDemand_);
    }
}
