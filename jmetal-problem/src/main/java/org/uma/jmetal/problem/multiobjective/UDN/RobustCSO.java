package org.uma.jmetal.problem.multiobjective.UDN;

import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.problem.multiobjective.UDN.model.DynamicUDN;

import java.util.Arrays;
import java.util.BitSet;
import java.util.List;

/**
 * Class representing problem ZDT1
 */
public class RobustCSO extends CSO {

    // The value for the H sample
    int samplingSize_;
    int currentSample_;

    //for updating the user positions
    boolean shanghai_ = false;

    /**
     * Creates an instance of the UDN planning problems.
     */
    public RobustCSO(String mainConfig, int run, int sampleSize) {

        //Create the UDN model
        udn_ = new DynamicUDN(mainConfig, run);

        bits = udn_.getTotalNumberOfActivableCells();

        setNumberOfVariables(1);
        setNumberOfObjectives(4);
        setNumberOfConstraints(0);
        setName("RobustCSO");
        run_ = run;
        samplingSize_ = sampleSize;

        //get mobility info for updating the users position
        if (udn_.getMobilityType().equalsIgnoreCase("shanghai")) {
            shanghai_ = true;
        }
    }

    /**
     * Evaluates a solution.
     *
     * @param solution The solution to evaluate.
     */
    @Override
    public BinarySolution evaluate(BinarySolution solution) {
        BitSet cso = solution.variables().get(0);

        //map the activation to the udn
        udn_.setCellActivation(cso);

        //update the avera
        udn_.computeSignaling();

        double sumCapacity = 0.0, sqSumCapacity = 0.0;
        double sumPower = 0.0, sqSumPower = 0.0;
        double capacity, powerConsumption;

        //Reset the sampling
        this.resetSampling();

        for (int t = 0; t < this.samplingSize_; t++) {
            capacity = networkCapacity(solution);
            powerConsumption = powerConsumptionPiovesan();

            sumCapacity += capacity;
            sqSumCapacity += capacity * capacity;

            sumPower += powerConsumption;
            sqSumPower += powerConsumption * powerConsumption;

            this.nextSample();

            //recompute signaling
            udn_.computeSignaling();
        }

        double meanCapacity = sumCapacity / this.samplingSize_;
        double varianceCapacity = (sqSumCapacity - (sumCapacity * sumCapacity) / this.samplingSize_) / (this.samplingSize_ - 1);

        double meanPower = sumPower / this.samplingSize_;
        double variancePower = (sqSumPower - (sumPower * sumPower) / this.samplingSize_) / (this.samplingSize_ - 1);

        solution.objectives()[0] = meanPower;
        solution.objectives()[1] = -variancePower;
        solution.objectives()[2] = -meanCapacity;
        solution.objectives()[3] = -varianceCapacity;

        return solution;

    } // evaluate

    public void nextSample() {
        this.currentSample_++;
        if (shanghai_)
            ((DynamicUDN) this.udn_).updateUsersPositionShanghai(this.currentSample_);
        else
            ((DynamicUDN) this.udn_).updateUsersPositionFromMatrix(this.currentSample_);

        //saving memory: recompute only interesiting points for the new epoch
        this.udn_.emptyMapsAtPoints();
    }

    private void resetSampling() {
        if (shanghai_)
            ((DynamicUDN) this.udn_).updateUsersPositionShanghai(0);
        else
            ((DynamicUDN) this.udn_).updateUsersPositionFromMatrix(0);

        //saving memory: recompute only interesiting points for the new epoch
        this.udn_.emptyMapsAtPoints();
    }

    @Override
    public List<Integer> getListOfBitsPerVariable() {
        return Arrays.asList(bits);
    }
} // Planning UDN
