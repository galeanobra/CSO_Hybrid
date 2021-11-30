package org.uma.jmetal.problem.multiobjective.UDN;

import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.problem.multiobjective.UDN.model.StaticUDN;
import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.BTS;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Cell;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Sector;

import java.util.Arrays;
import java.util.BitSet;
import java.util.List;

/**
 * Class representing problem ZDT1
 */
public class StaticCSO extends CSO {

    /**
     * Creates an instance of the Static CSO problem
     */
    public StaticCSO(String mainConfig, int run) {

        //Create the UDN model
        udn_ = new StaticUDN(mainConfig, run);

        bits = udn_.getTotalNumberOfActivableCells();

        setNumberOfVariables(1);
        setNumberOfObjectives(2);
        setNumberOfConstraints(0);
        setName("StaticCSO");

        udn_.getTotalNumberOfActivableCells();

        run_ = run;

        //load operators config
        loadOperatorsConfig(udn_.getOperatorsFile());

        //udn_.printXVoronoi();
        //System.exit(-1);
    }

    public StaticCSO(String problemconf) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    public StaticCSO(String mainconf, int run, int epochs) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }


    /**
     * Evaluates a solution.
     *
     * @param solution The solution to evaluate.
     */
    @Override
    public BinarySolution evaluate(BinarySolution solution) {
        BitSet cso = solution.variables().get(0);

        boolean noActiveCells = true;
        for (int i = 0; i < cso.length(); i++) {
            if (cso.get(i)) {
                noActiveCells = false;
                break;
            }
        }

        if (!noActiveCells) {
            //map the activation to the udn
            udn_.setCellActivation(cso);

            //update the avera
            udn_.computeSignaling();

            double capacity = networkCapacity(solution);
            double powerConsumption = powerConsumptionStatic();
            solution.objectives()[0] = powerConsumption;
            solution.objectives()[1] = -capacity;
//            System.out.println(powerConsumption + " " + capacity);
        } else {
            solution.objectives()[0] = 0.0;
            solution.objectives()[1] = 0.0;
        }

        return solution;
    } // evaluate

    /**
     * m
     * In this function operators are applied in order to improve the sinr of
     * certain problematic points in the network by switching off some BTS
     *
     * @param solution: Solution to be modified
     */
    @Override
    public void intelligentSwitchOff(BinarySolution solution) {
//        Map<Double, List<Point>> worsePoints = new TreeMap<>();
//        double sinr_limit = 12;
//
//        for (double op_frequency : this.udn_.cells_.keySet()) {
//            List<Point> l = new ArrayList<>();
//            for (User u : this.udn_.getUsers()) {
//                Cell c = u.getServingCell();
//                double f = c.getBTS().getWorkingFrequency();
//                if (Double.compare(f, op_frequency) == 0) {
//                    Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
//                    if (p.computeSINR(c) < sinr_limit) {
//                        l.add(p);
//                    }
//                }
//            }
//            if (!l.isEmpty()) {
//                l = sortList(l);
//                worsePoints.put(op_frequency, l);
//            }
//        }

        //apply operators
        // noUsersOp(solution);
        //macro1Op(worsePoints);
        //macro2Op(worsePoints);
        //tooManyUsersOp();
        //priorizeFemtoOp();

        if (this.operators_.containsKey("maintenancePowerOp"))
            maintenancePowerOp(this.operators_.get("maintenancePowerOp"), solution);

        if (this.operators_.containsKey("noUsersOp"))
            noUsersOp(this.operators_.get("noUsersOp"), solution);

        if (this.operators_.containsKey("priorizeSmallCellsOp"))
            priorizeSmallCellsOp(this.operators_.get("priorizeSmallCellsOp"), solution);

        if (this.operators_.containsKey("priorizeFemtoOp"))
            priorizeFemtoOp(this.operators_.get("priorizeFemtoOp"), solution);

        if (this.operators_.containsKey("increaseCapacityOp"))
            increaseCapacityOp(this.operators_.get("increaseCapacityOp"), solution);

        //priorizeSmallCellsOp(this.operators_.get("priorizeSmallCellsOp"), solution);
        //modifySolution(solution);
    }

    /**
     * Calculates the power consumption taking into account the total traffic demand
     * and the maintenance power, in the case of small cells (pico, femto)
     *
     * @return Power consumption
     */
    double powerConsumptionStatic() {
        double sum = 0.0;
        boolean hasActiveCells;
        double maintenancePower = 2000; //mW

        for (List<BTS> btss : udn_.btss_.values()) {
            for (BTS bts : btss) {
                hasActiveCells = false;
                for (Sector sector : bts.getSectors()) {
                    for (Cell cell : sector.getCells()) {
                        if (cell.isActive()) {
                            hasActiveCells = true;
                            sum += sector.getTransmittedPower() * sector.getAlfa() + sector.getBeta() + sector.getDelta() * cell.getTrafficDemand() + 10;
                        } else {
                            //residual consuption in sleep mode (mW)
                            sum += sector.getTransmittedPower() * 0.01;
                        }
                    }
                }
                if (hasActiveCells) {
                    sum += maintenancePower;
                }
            }
        }

        //mW -> W -> kW -> MW
        sum /= 1000000000;
        //System.out.println("Consumed power = " + sum);

        return sum;
    }// powerConsumptionStatic

    public UDN getUDN() {
        return udn_;
    }

    @Override
    public List<Integer> getListOfBitsPerVariable() {
        return Arrays.asList(bits);
    }
} // Planning UDN
