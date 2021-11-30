package org.uma.jmetal.problem.multiobjective.UDN.model.users;

/**
 * @author paco
 */
public abstract class MobilityModel {

    abstract public void move(User u);

    public abstract double getMinV_();

    public abstract double getMaxV_();
}
