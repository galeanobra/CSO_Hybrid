package org.uma.jmetal.problem.multiobjective.UDN.model;

/**
 * @author paco
 */
class Region {
    int id_;
    int x_;
    int y_;

    //Propagation parameters
    //long seed_;
    double pathloss_;
    double mean_; //mean value for the random variable
    double std_db_; //standard deviation for the random variable
    double kd_;
    int channelType_;

    /**
     * Parametrized constructor
     *
     * @param x_ Coordinate x of the region attraction point
     * @param y_ Coordinate y of the region attraction point
     */
    public Region(int id, int x_, int y_) {
        this.id_ = id;
        this.x_ = x_;
        this.y_ = y_;

        //generates a pathloss between 2 and 4
        this.pathloss_ = 2.0 + 2.0 * UDN.random_.nextDouble();
        //generates a mean and std for the random variable
        this.mean_ = UDN.random_.nextDouble();
        this.std_db_ = 0.5 + 2.0 * UDN.random_.nextDouble();
        this.kd_ = 20.0 * UDN.random_.nextDouble();
        this.channelType_ = 1 + UDN.random_.nextInt(5);
    }
}
