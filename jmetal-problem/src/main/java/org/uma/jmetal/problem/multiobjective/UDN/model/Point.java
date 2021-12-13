package org.uma.jmetal.problem.multiobjective.UDN.model;

import org.uma.jmetal.problem.multiobjective.UDN.model.cells.BTS;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Cell;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Sector;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Cell;

import java.util.*;

/**
 * @author paco
 */
public class Point {

    //Reference to the problem
    UDN udn_;

    //Point coordinates
    int x_;
    int y_;
    int z_;

    Map<Double, BTS> installedBTS_;

    //Propagation region
    Region propagationRegion_;

    //SINR
    Map<Double, Double> totalReceivedPower_;
//    Map<Double, Cell> sinr_;

    //Used to precompute stats only when needed_
    boolean statsComputed_;

    //Function<Cell, Double> function_;

    Map<Integer, Double> signalPowerMap_ = new HashMap<>();
    Map<Integer, Double> sinrMap_ = new HashMap<>();

    boolean canDeploy;

    /**
     * Constructor
     *
     * @param x Coordinate x
     * @param y Coordinate y
     * @param z Coordinate z
     */
    Point(UDN udn, int x, int y, int z) {
        udn_ = udn;
        x_ = x;
        y_ = y;
        z_ = z;

        installedBTS_ = new TreeMap<>();
        propagationRegion_ = null;
        totalReceivedPower_ = null;
        statsComputed_ = false;
    }

    Region getPropagationRegion() {
        return this.propagationRegion_;
    }

    void setPropagationRegion(Region r) {
        propagationRegion_ = r;
    }

    /**
     * Computes the signal power received at this grid point from the Cell c
     *
     * @param c The serving Cell
     * @return The received power
     */
    public double computeSignalPower(Cell c) {
        double powerDBm;

        //create a new map after X insertions to bound the memory used
        if (signalPowerMap_.get(c.getID()) == null) {
            Sector sec = c.getSector();
            double pathLoss = this.propagationRegion_.pathloss_;
            double receptorGain = Math.pow(10.0, sec.getReceptorGain() / 10.0);
            double transmitterGain = Math.pow(10.0, sec.getTransmitterGain() / 10.0);
            double waveLength = c.getWavelength();
            double transmitterPower = sec.getTransmittedPower();
            double distance = this.udn_.distance(this.x_, this.y_, this.z_, sec.getX(), sec.getY(), sec.getZ());
            double power;
            int[] angles = this.udn_.calculateAngles(this, sec.getBTS());
            double attenuationFactor = c.getAttenuationFactor(angles[0], angles[1]);
            double loss = Math.pow((waveLength / (4.0 * Math.PI * distance)), pathLoss);
            power = receptorGain * transmitterGain * transmitterPower * loss * attenuationFactor;
            //
            powerDBm = 10.0 * Math.log10(power);
            signalPowerMap_.put(c.getID(), powerDBm);
        } else {
            powerDBm = signalPowerMap_.get(c.getID());
        }

        return powerDBm;
    }

    /**
     * Returns the closest social attractor to this point
     *
     * @return Closest SA
     */
    SocialAttractor getClosestSA(UDN udn) {

        SocialAttractor sa = null;
        double minDistance = Double.MAX_VALUE;
        double d;

        for (SocialAttractor s : udn.socialAttractors_) {
            d = udn.distance(this.x_, this.y_, this.z_, s.getX(), s.getY(), s.getZ());
            if (d < minDistance) {
                minDistance = d;
                sa = s;
            }
        }

        return sa;
    }

    /**
     * Distance to the BTS of the given cell.
     *
     * @param c Cell
     * @return The Euclidean distance from this point to the Cell BTS
     */
    private double distanceToBTS(Cell c) {
        return this.udn_.distance(this.x_, this.y_, this.z_, c.getBTS().getX(), c.getBTS().getY(), c.getBTS().getZ());
    }

    /**
     * Computes the received SINR from a given cell
     *
     * @param c Cell
     * @return SINR received by c
     */
    public double computeSINR(Cell c) {

        //get the bandwidth of the Cell and its working frequency
        double cellBW = c.getTotalBW();
        double frequency = c.getBTS().getWorkingFrequency();

        //compute the noise
        double pn = -174 + 10.0 * Math.log10(cellBW * 1000000);

        //get the averaged power received at the BTS working frequency
        double totalPower = this.totalReceivedPower_.get(frequency);

        //compute the power received at this point from Cell c
        double power = this.computeSignalPower(c);

        //double distance = this.udn_.distance(this.x_, this.y_, c.getBTS().getX(), c.getBTS().getY());
        //compute the SINR
        //dB -> mW

        pn = Math.pow(10.0, pn / 10);
        power = Math.pow(10.0, power / 10);

        return power / (totalPower - power + pn);
    }

    /**
     * Precomputes the averaged SINR at each grid point, for each
     */
    void computeTotalReceivedPower() {
        //allocate memory at this point
        totalReceivedPower_ = new TreeMap<>();

        double sum, power;

        for (double frequency : this.udn_.cells_.keySet()) {
            sum = 0.0;

            for (Cell c : this.udn_.cells_.get(frequency)) {
                if (c.isActive()) {

                    power = computeSignalPower(c);
                    //dB -> mW
                    power = Math.pow(10.0, power / 10);
                    sum += power;
                }
            }

            this.totalReceivedPower_.put(frequency, sum);
        }
    }

    /**
     * Return the closest BTS in terms of the received signal power. Required by
     * M. Mirahsan, R. Schoenen, and H. Yanikomeroglu, “HetHetNets:
     * Heterogeneous Traffic Distribution in Heterogeneous Wireless Cellular
     * Networks,” IEEE J. Sel. Areas Commun., vol. 33, no. 10, pp. 2252–2265,
     * 2015.
     *
     * @return The closest BTS
     */
    Cell getCellWithHigherReceivingPower() {
        double power;
        double maxPower = Double.NEGATIVE_INFINITY;
        Cell closest = null;

        for (Double frequency : this.udn_.cells_.keySet()) {
            for (Cell c : udn_.cells_.get(frequency)) {
                power = this.computeSignalPower(c);
                if (power > maxPower) {
                    maxPower = power;
                    closest = c;
                }
            }
        }

        return closest;
    }

    /**
     * Returns the cell that serves with the best SINR, regardless of its
     * operating frequency.
     *
     * @return The cell with higher SINR
     */
    public Cell getCellWithHigherSINR() {
        double sinr;
        Map<Double, Double> maxSINR = new TreeMap<>();
        Map<Double, Cell> servingBTSs = new TreeMap<>();
        Cell servingCell = null;

        for (Double frequency : this.udn_.cells_.keySet()) {
            maxSINR.put(frequency, Double.NEGATIVE_INFINITY);
        }

        for (Double frequency : this.udn_.cells_.keySet()) {
            for (Cell c : udn_.cells_.get(frequency)) {
                if (c.isActive()) {
                    sinr = this.computeSINR(c);

                    //quality, regardless of the cell activation
                    if (sinr > maxSINR.get(frequency)) {
                        maxSINR.put(frequency, sinr);
                        servingBTSs.put(frequency, c);
                    }
                }
            }
        }

        // If using cellsOfInterestByPoint in UDN
//        for (double frequency : this.udn_.cellsOfInterestByPoint.get(this).keySet()) {
//            for (Cell c : this.udn_.cellsOfInterestByPoint.get(this).get(frequency)) {
//                if (c.isActive()) {
//                    sinr = this.computeSINR(c);
//                    if (sinr > maxSINR.get(frequency)) {
//                        maxSINR.put(frequency, sinr);
//                        servingBTSs.put(frequency, c);
//                    }
//                }
//            }
//        }

        //retrieve the best among the precomputed values
        double maxValue = Double.NEGATIVE_INFINITY;
        for (Double f : servingBTSs.keySet()) {
            sinr = maxSINR.get(f);
            if (sinr > maxValue) {
                maxValue = sinr;
                servingCell = servingBTSs.get(f);
            }
        }

        return servingCell;
    }

    /**
     * Returns the cell that serves with the best SINR, discarding macrocells
     * and regardless of its operating frequency.
     *
     * @return The cell with higher SINR discarding macrocells
     */
    public Cell getCellWithHigherSINRButMacro() {
        double sinr;
        Map<Double, Double> maxSINR = new TreeMap<>();
        Map<Double, Cell> servingBTSs = new TreeMap<>();
        Cell servingCell = null;

        for (Double frequency : this.udn_.cells_.keySet()) {
            maxSINR.put(frequency, Double.NEGATIVE_INFINITY);
        }

        for (Double frequency : this.udn_.cells_.keySet()) {
            for (Cell c : udn_.cells_.get(frequency)) {

                if (c.isActive() && !(c.getType().toString().equalsIgnoreCase("MACRO"))) {
                    sinr = this.computeSINR(c);

                    //quality, regardless of the cell activation
                    if (sinr > maxSINR.get(frequency)) {
                        maxSINR.put(frequency, sinr);
                        servingBTSs.put(frequency, c);
                    }
                }
            }
        }//for

        //retrieve the best among the precomputed values
        double maxValue = Double.NEGATIVE_INFINITY;
        for (Double f : servingBTSs.keySet()) {
            sinr = maxSINR.get(f);
            if (sinr > maxValue) {
                maxValue = sinr;
                servingCell = servingBTSs.get(f);
            }
        }

        return servingCell;
    }

    /**
     * Returns the small cell (femto or pico) that serves with the best SINR,
     *
     * @return The small cell with higher SINR
     */
    public Cell getSmallCellWithHigherSINR() {
        double sinr;
        Map<Double, Double> maxSINR = new TreeMap<>();
        Map<Double, Cell> servingBTSs = new TreeMap<>();
        Cell servingCell = null;

        for (Double frequency : this.udn_.cells_.keySet()) {
            maxSINR.put(frequency, Double.NEGATIVE_INFINITY);
        }

        for (Double frequency : this.udn_.cells_.keySet()) {
            for (Cell c : udn_.cells_.get(frequency)) {

                if (c.isActive() && ((c.getType().toString().equalsIgnoreCase("PICO")) || (c.getType().toString().equalsIgnoreCase("FEMTO")))) {
                    sinr = this.computeSINR(c);

                    //quality, regardless of the cell activation
                    if (sinr > maxSINR.get(frequency)) {
                        maxSINR.put(frequency, sinr);
                        servingBTSs.put(frequency, c);
                    }
                }
            }
        }//for

        //retrieve the best among the precomputed values
        double maxValue = Double.NEGATIVE_INFINITY;
        for (Double f : servingBTSs.keySet()) {
            sinr = maxSINR.get(f);
            if (sinr > maxValue) {
                maxValue = sinr;
                servingCell = servingBTSs.get(f);
            }
        }

        return servingCell;
    }

    /**
     * Return a sorted list of Cells with the best serving SINR, regardless of
     * their operation frequency and type
     *
     * @return Sorted cells
     */
    public SortedMap<Double, Cell> getCellsWithBestSINRs() {
        //create the comparator for the sortedlist
        Comparator<Double> cellSINRComparator = (sinr1, sinr2) -> Double.compare(sinr2, sinr1);

        SortedMap<Double, Cell> sortedCells = new TreeMap<>(cellSINRComparator);

        for (Double frequency : this.udn_.cells_.keySet()) {
            for (Cell c : udn_.cells_.get(frequency)) {
                sortedCells.put(this.computeSINR(c), c);
            }
        }

        return sortedCells;
    }

    Map<Double, Double> getTotalReceivedPower() {
        return this.totalReceivedPower_;
    }

    public boolean hasBTSInstalled() {
        return !installedBTS_.isEmpty();
    }

    public boolean hasBTSInstalled(double workingFrequency) {
        return installedBTS_.containsKey(workingFrequency);
    }

    public void addInstalledBTS(double workingFrequency, BTS bts) {
        if (!installedBTS_.containsKey(workingFrequency)) {
            installedBTS_.put(workingFrequency, bts);
        }
    }

    public Map<Double, BTS> getInstalledBTS() {
        return this.installedBTS_;
    }

    public List<Cell> getCells() {
        List<Cell> cells = new ArrayList<>();

        installedBTS_.keySet().forEach(d -> installedBTS_.get(d).getSectors().forEach(sector -> cells.addAll(sector.getCells())));

        return cells;
    }

    public List<Cell> getActiveCells() {
        List<Cell> activeCells = new ArrayList<>();

        for (double d : installedBTS_.keySet()) {
            for (Sector sector : installedBTS_.get(d).getSectors()) {
                for (Cell cell : sector.getCells()) {
                    if (cell.isActive())
                        activeCells.add(cell);
                }
            }
        }

        return activeCells;
    }

    public int[] getPoint2D() {
        return new int[]{this.x_, this.y_};
    }
}
