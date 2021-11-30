package org.uma.jmetal.problem.multiobjective.UDN.simulation;

import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;
import org.uma.jmetal.problem.multiobjective.UDN.model.users.User;
import org.uma.jmetal.util.PPP;

import java.util.List;

/**
 * @author paco
 */
public class Simulation {
    //simulation parameters
    UDN udn_;
    int t_;
    double tics_;
    int simulationTime_;

    /**
     * Parametrized constructor
     *
     * @param udn
     * @param simulationTime
     * @param tics
     */
    public Simulation(UDN udn, int simulationTime, double tics) {
        this.udn_ = udn;
        this.simulationTime_ = simulationTime;
        this.tics_ = tics;

        //initial user deployment
        initialUserPlacement();
    }

    public void run() {

    }

    private void initialUserPlacement() {
        List<User> users = this.udn_.getUsers();

        //create independent Poison Point Processes for deploying
        //the BTSs
        PPP btsPPP = new PPP(UDN.random_);
        //the social attractors
        PPP saPPP = new PPP(UDN.random_);
        //the users
        PPP userPPP = new PPP(UDN.random_);

        //deploy BTSs
        //deploy(btsPPP, 100);
    }
}
