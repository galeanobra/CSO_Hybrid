package org.uma.jmetal.problem.multiobjective.UDN.model.users;

import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

public class RandomWaypoint extends MobilityModel {

    //velocity in m/s
    private double minV_;
    private double maxV_;
    private final UDN udn;

    public RandomWaypoint(UDN udn, String configFile) {
        this.udn = udn;

        Properties pro = new Properties();

        try {
            System.out.println("Loading Random Waypoint model config file...");
            pro.load(new FileInputStream(configFile));

            minV_ = Double.parseDouble(pro.getProperty("minVelocity"));
            maxV_ = Double.parseDouble(pro.getProperty("maxVelocity"));

        } catch (IOException e) {
            System.out.println(e + "Error loading properties: " + configFile);
            System.exit(-1);
        }
    }

    @Override
    public void move(User u) {
        int[] actual = new int[3]; // actual[0] -> x; actual[1] -> y; actual[2] -> z

        int minutes = 30;   // Minutes per epoch

        for (int i = 0; i < minutes; i++) {
            int d_x = (int) ((int) (u.getVelocity()[0] * 60) / udn.getInterpointSeparation());    // distance covered in x axis in 1 minute
            int d_y = (int) ((int) (u.getVelocity()[1] * 60) / udn.getInterpointSeparation());    // distance covered in y axis in 1 minute
//        int d_z = (int) ((int) (u.getVelocity()[2] * 60) / udn.getInterpointSeparation());  // distance covered in z axis in 1 minute

            actual[0] = u.getX_t() >= u.getX() ? Math.min(u.getX() + d_x, u.getX_t()) : Math.max(u.getX() - d_x, u.getX_t());
            actual[1] = u.getY_t() >= u.getY() ? Math.min(u.getY() + d_y, u.getY_t()) : Math.max(u.getY() - d_y, u.getY_t());
//        actual[2] = u.getZ_t() >= u.getZ() ? Math.min(u.getZ() + d_z, u.getZ_t()) : Math.max(u.getZ() - d_z, u.getZ_t());
            actual[2] = 0;

            if (targetPosition(u, actual)) {    // If the user arrives the destination point
                u.setVelocity(new double[]{(maxV_ - minV_) * UDN.random_.nextDouble() + minV_, (maxV_ - minV_) * UDN.random_.nextDouble() + minV_, (maxV_ - minV_) * UDN.random_.nextDouble() + minV_});

                u.setX_t(udn.getRandom().nextInt(udn.getGrid().length));
                u.setY_t(udn.getRandom().nextInt(udn.getGrid()[0].length));
//            u.setZ_t(udn.getRandom().nextInt(udn.getGrid()[0][0].length));
                u.setZ_t(0);

                u.setX(u.getX_t());
                u.setY(u.getY_t());
                u.setZ(u.getZ_t());
            } else {    // If the user is moving towards the target point
                u.setX(actual[0]);
                u.setY(actual[1]);
                u.setZ_t(actual[2]);
            }
        }
    }

    public boolean targetPosition(User u, int[] actual) {
        return actual[0] == u.getX_t() && actual[1] == u.getY_t() && actual[2] == u.getZ_t();
    }

    public double getMinV_() {
        return minV_;
    }

    public void setMinV_(double minV_) {
        this.minV_ = minV_;
    }

    public double getMaxV_() {
        return maxV_;
    }

    public void setMaxV_(double maxV_) {
        this.maxV_ = maxV_;
    }
}
