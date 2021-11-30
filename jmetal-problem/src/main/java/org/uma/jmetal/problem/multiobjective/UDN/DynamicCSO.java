package org.uma.jmetal.problem.multiobjective.UDN;

import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.problem.multiobjective.UDN.model.DynamicUDN;
import org.uma.jmetal.problem.multiobjective.UDN.model.Point;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.BTS;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Cell;

import java.util.Arrays;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

/**
 * Class representing problem ZDT1
 */
public class DynamicCSO extends CSO {

    int evaluations_ = 0;
    int epochs_;
    int currentEpoch_ = 0;

    //for updating the user positions
    boolean shanghai_ = false;

    /**
     * Creates an instance of the UDN planning problems.
     */
    public DynamicCSO(String mainConfig, int run, int epochs) {

        //Create the UDN model
        udn_ = new DynamicUDN(mainConfig, run);

        bits = udn_.getTotalNumberOfActivableCells();

        setNumberOfVariables(1);
        setNumberOfObjectives(3);
        setNumberOfConstraints(0);
        setName("DynamicCSO");
        run_ = run;
        epochs_ = epochs;

        //get mobility info for updating the users position
        if (udn_.getMobilityType().equalsIgnoreCase("shanghai"))
            shanghai_ = true;
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

            //It has to be called before as it stores the current
            // UEs to Cell assignment and stores it in the solution
            double capacity = networkCapacity(solution);
            int handovers = incurredHandovers(solution);
            double powerConsumption = powerConsumptionPiovesan();
            double signalingCost = signalingCost(solution);

            solution.objectives()[0] = powerConsumption;
            solution.objectives()[1] = -capacity;
            //solution.setObjective(2, signalingCost);
            // solution.setObjective(3, signalingCost);
        } else {
            solution.objectives()[0] = 0.0;
            solution.objectives()[1] = 0.0;
            //solution.setObjective(2, 0);
            //solution.setObjective(3, 0);
        }

        return solution;
    } // evaluate


    public int getNumberOfEpochs() {
        return this.epochs_;
    }

    public void nextEpoch() {
        this.currentEpoch_++;
        if (udn_.getMobilityType().equalsIgnoreCase("shanghai"))
            ((DynamicUDN) this.udn_).updateUsersPositionShanghai(this.currentEpoch_);
        else if (udn_.getMobilityType().equalsIgnoreCase("userDemands") || udn_.getMobilityType().equalsIgnoreCase("matrix"))
            ((DynamicUDN) this.udn_).updateUsersPositionFromMatrix(this.currentEpoch_);
        else    // Random Waypoint
            ((DynamicUDN) this.udn_).updateUsersPositionFromModel();

        //update the accumulated evaluations on each epoch
        evaluations_ = 0;

        //saving memory: recompute only interesiting points for the new epoch
        this.udn_.emptyMapsAtPoints();
    }

    private int incurredHandovers(BinarySolution solution) {
        int handovers = 0;

        List<Cell> previous = solution.getPreviousUesToCellAssignment();
        if ((currentEpoch_ > 0) && (previous != null)) {

            List<Cell> current = solution.getCurrentUesToCellAssignment();

            for (int i = 0; i < current.size(); i++) {

                if (current.get(i).getID() != previous.get(i).getID())
                    //There is a handover 
                    handovers++;
            }
        } else
            handovers = this.udn_.getUsers().size();

        return handovers;
    }

    /**
     * Calculates the total signaling cost of the HetNet, taking into account different handover types:
     * Source:
     * Evolutionary 4G/5G Network Architecture Assisted Efficient Handover Signaling
     * Mobility Management in 5G-enabled Vehicular Networks: Models, Protocols, and Classification
     *
     * @param solution Solution
     * @return Signaling cost
     */
    private float signalingCost(BinarySolution solution) {
        //Vertical
        float cost_intraTower = 113; //ms
        //Horizontal
        float cost_interTower = 159; //ms

        float cost_interTech = 82; //ms
        float totalCost = 0;

        List<Cell> previous = solution.getPreviousUesToCellAssignment();

        if ((currentEpoch_ > 0) && (previous != null)) {

            List<Cell> current = solution.getCurrentUesToCellAssignment();

            for (int i = 0; i < current.size(); i++) {
                int currentID = current.get(i).getID(); //ID of previous assigned cell
                int previousID = previous.get(i).getID(); //ID of current assigned cell
                BTS currentBts = current.get(i).getBTS();
                BTS previousBts = previous.get(i).getBTS();
                if (currentID != previousID) {
                    //There is a handover
                    if (currentBts.getX() == previousBts.getX() && currentBts.getY() == previousBts.getY()) {
                        //same tower
                        totalCost += cost_intraTower;

                    } else {
                        //different tower
                        totalCost += cost_interTower;

                    }
                    if (currentBts.getWorkingFrequency() != previousBts.getWorkingFrequency()) {
                        //change of technology
                        totalCost += cost_interTech;
                    }

                }
            }
        } else
            totalCost = this.udn_.getUsers().size();

        return totalCost;
    }


    public void adjacentBtsRestart(BinarySolution s, int length, double rate) {
        BitSet cso = s.variables().get(0);

        //map the activation to the udn
        udn_.setCellActivation(cso);

        //invoke network capacity to allocate users to cells
        udn_.computeSignaling();
        networkCapacity(s);

        //forget previous assignent
        s.forgetUEsToCellAssignment();

        //go through all the cells which does have users connected, and restart adyacent BTSs
        Random r = new Random();
        for (List<Cell> cells : udn_.cells_.values()) {
            for (Cell c : cells) {
                if (c.getAssignedUsers() > 0) {
                    BTS b = c.getBTS();
                    int x = b.getX();
                    int y = b.getY();
                    int z = b.getZ();

                    //activate BTSs within "length" points distances
                    for (int l = 1; l <= length; l++) {
                        for (int deltaX = -l; deltaX <= l; deltaX++) {
                            //avoid activating the current BTS
                            for (int deltaY = -l; deltaY <= l; deltaY++) {
                                for (int deltaZ = -l; deltaZ <= l; deltaZ++) {
                                    if ((deltaX != 0) || (deltaY != 0) || (deltaZ != 0)) {
                                        //randomize the decision
                                        if (r.nextDouble() < rate) {
                                            //check array indexses
                                            int locX = x + deltaX;
                                            int locY = y + deltaY;
                                            int locZ = z + deltaZ;
                                            if (checkLimitX(locX) && checkLimitY(locY) && checkLimitZ(locZ)) {
                                                Point p = udn_.getGridPoint(locX, locY, locZ);
                                                if (p.hasBTSInstalled()) {
                                                    // Qué celda activar?
//                                                    p.getInstalledCell().setActivation(true); //TODO
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        modifySolution(s);
    }

    private boolean checkLimitX(int v) {
        return (v >= 0) && (v < udn_.gridPointsX_);
    }

    private boolean checkLimitY(int v) {
        return (v >= 0) && (v < udn_.gridPointsY_);
    }

    private boolean checkLimitZ(int v) {
        return (v >= 0) && (v < udn_.gridPointsZ_);
    }

    @Override
    public List<Integer> getListOfBitsPerVariable() {
        return Arrays.asList(bits);
    }
}
