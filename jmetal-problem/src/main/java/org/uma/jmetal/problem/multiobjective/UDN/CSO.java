package org.uma.jmetal.problem.multiobjective.UDN;

import org.uma.jmetal.problem.binaryproblem.impl.AbstractBinaryProblem;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.problem.multiobjective.UDN.model.Point;
import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;
import org.uma.jmetal.problem.multiobjective.UDN.model.UDN.CellType;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.BTS;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Cell;
import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Sector;
import org.uma.jmetal.problem.multiobjective.UDN.model.users.User;
import org.uma.jmetal.util.errorchecking.JMetalException;
import org.uma.jmetal.util.pseudorandom.impl.JavaRandomGenerator;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.*;

import static org.uma.jmetal.problem.multiobjective.UDN.model.UDN.CellType.*;

/**
 * Class representing problem ZDT1
 */
public abstract class CSO extends AbstractBinaryProblem {

    /**
     * Number of activable cells
     */
    protected int bits;

    /**
     * The underlying UDN
     */
    protected UDN udn_;

    /**
     * Operator Configuration
     */
    protected Map<String, Double> operators_; //operators with their applications rate

    /**
     * The seed to generate the instance
     */
    protected int run_;

    public int getTotalNumberOfActivableCells() {
        return udn_.getTotalNumberOfActivableCells();
    }

    int pointsWithStatsComputed() {
        return udn_.pointsWithStatsComputed();
    }

    double powerConsumptionBasedOnTransmittedPower() {
        double sum = 0.0;

        for (List<Cell> cells : udn_.cells_.values()) {
            for (Cell c : cells) {
                if (c.isActive()) {
                    sum += 4.7 * c.getSector().getTransmittedPower();
                } else {
                    //residual consuption in sleep mode (mW)
                    sum += 160;
                }
            }
        }
        //mW -> W -> kW -> MW
        sum /= 1000000000;

        return sum;
    }

    /**
     * Calculates the power consumption taking into account the total traffic demand
     *
     * @return Power consumption
     */
    double powerConsumptionPiovesan() {
        double sum = 0.0;

        for (List<Cell> cells : udn_.cells_.values()) {
            for (Cell c : cells) {
                Sector sector = c.getSector();
                if (c.isActive()) {
                    //sum += c.getBTS().getBaseConsumedPower() * 4.7 * c.getBTS().getTransmittedPower();
//                    double td = c.getTrafficDemand();
                    sum += sector.getTransmittedPower() * sector.getAlfa() + sector.getBeta() + sector.getDelta() * c.getTrafficDemand() + 10;
                } else {
                    //residual consuption in sleep mode (mW)
                    sum += sector.getTransmittedPower() * 0.01;
                }
            }
        }
        //mW -> W -> kW -> MW
        sum /= 1000000000;
        //System.out.println("Consumed power = " + sum);
        return sum;
    }

    double[][] loadH(BTS bts) {
        return null;
    }

    void saveCellStatus(BinarySolution s) {
        BitSet cso = s.variables().get(0);

        //map the activation to the udn
        udn_.copyCellActivation(cso);
    }

    /**
     * Max capacity of the 5G network. At each point, it returns the best SINR
     * for each of the different operating frequencies.
     *
     * @return Network capacity
     */
    double networkCapacity(BinarySolution solution) {
        /*
          For the dynamic problem addressing
         */
        List<Cell> assignment = new ArrayList<>();

        double capacity = 0.0;

        //0.- Reset number of users assigned to cells
        udn_.resetNumberOfUsersAssignedToCells();

        //1.- Assign users to cells, to compute the BW allocated to them
        for (User u : this.udn_.getUsers()) {
            u.setServingCell(udn_.getGridPoint(u.getX(), u.getY(), u.getZ()).getCellWithHigherSINR());
            u.getServingCell().addUserAssigned();

            //dynamic
            // assignment.add(u.getServingCell().getID());
            assignment.add(u.getServingCell());
        }

        //save the assignment into the solution
        solution.setUEsToCellAssignment(assignment);

        //1.- computes the Mbps allocated to each user
        for (User u : this.udn_.getUsers()) {
            double allocatedBW = u.getServingCell().getSharedBWForAssignedUsers();

            //computes the Mbps
            //double c = u.capacity(this.udn_, allocatedBW);
            double c = u.capacityMIMO(this.udn_, allocatedBW);
            capacity += c / 1000.0;
        }

        //udn_.validateUserAssigned();
        return capacity;
    }

    public int getRun() {
        return this.run_;
    }

    private double numberOfActiveCells() {
        int count = 0;

        for (List<Cell> cells : udn_.cells_.values()) {
            for (Cell c : cells) {
                if (c.isActive()) {
                    count++;
                }

            }
        }

        return count;
    }

    /**
     * In this function operators are applied in order to improve the sinr of
     * certain problematic points in the network by switching off some BTS
     */
    public void intelligentSwitchOff(BinarySolution solution) throws JMetalException {
        Map<Double, List<Point>> worsePoints = new HashMap<>();
        double sinr_limit = 12;

        for (double op_frequency : this.udn_.cells_.keySet()) {
            List<Point> l = new ArrayList<>();
            for (User u : this.udn_.getUsers()) {
                Cell c = u.getServingCell();
                double f = c.getBTS().getWorkingFrequency();
                if (Double.compare(f, op_frequency) == 0) {
                    Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                    if (p.computeSINR(c) < sinr_limit) {
                        l.add(p);
                    }
                }
            }
            if (!l.isEmpty()) {
                l = sortList(l);
                worsePoints.put(op_frequency, l);
            }
        }

        //apply operators
        // noUsersOp(solution);
        //macro1Op(worsePoints);
        //macro2Op(worsePoints);
        //tooManyUsersOp();
        //priorizeFemtoOp();
        modifySolution(solution);
    }

    /**
     * Sort a given list of Points by it SINR, being the worse the first
     *
     * @param l : list to sort
     * @return sorted list
     */
    public List<Point> sortList(List<Point> l) {
        double[] sinr_list = new double[l.size()];
        List<Point> sortedList = new ArrayList<>();
        double min_sinr = 5;

        for (int i = 0; i < l.size(); i++) {
            Point p = l.get(i);
            Cell c = p.getCellWithHigherSINR();
            double sinr = p.computeSINR(c);
            sinr_list[i] = sinr;

        }
        Arrays.sort(sinr_list);
        int index = 0;
        for (int i = 0; i < l.size(); i++) {
            for (int j = 0; j < l.size(); j++) {
                Point p_ = l.get(j);
                Cell c_ = p_.getCellWithHigherSINR();
                double sinr_ = p_.computeSINR(c_);
                if (Double.compare(sinr_, sinr_list[i]) == 0) {
                    index = j;
                    break;
                }
            }
            sortedList.add(i, l.get(index));
        }
        return sortedList;
    }

    /**
     * Cells with no users assigned are switched off.
     *
     * @param rate     : Application rate
     * @param solution The solution to be modified.
     */
    public void noUsersOp(double rate, BinarySolution solution) {
        if (new JavaRandomGenerator().nextDouble() < rate) {
            BitSet cso = solution.variables().get(0);

            udn_.setCellActivation(cso);
            udn_.computeSignaling();
            udn_.resetNumberOfUsersAssignedToCells();

            if (udn_.getTotalNumberOfActiveCells() > 0) {

                //Assign users to cells, to compute the BW allocated to them
                for (User u : this.udn_.getUsers()) {
                    Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                    Cell c = p.getCellWithHigherSINR();
                    c.addUserAssigned();
                    u.setServingCell(c);
                }

                for (double frequency : this.udn_.cells_.keySet()) {
                    for (Cell c : udn_.cells_.get(frequency)) {
                        if (c.getAssignedUsers() == 0)
                            c.setActivation(false);
                    }
                }

                modifySolution(solution);
            }
            // System.out.println("The noUsersOp has turned off "+count+" cells");
        }//if
    }

    public void increaseCapacityOp(double rate, BinarySolution solution) {
        if (new JavaRandomGenerator().nextDouble() < rate) {
            BitSet cso = solution.variables().get(0);

            udn_.setCellActivation(cso);
            udn_.computeSignaling();
            udn_.resetNumberOfUsersAssignedToCells();

            if (udn_.getTotalNumberOfActiveCells() > 0) {

                // Assign users to cells
                for (User u : this.udn_.getUsers()) {
                    Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                    Cell c = p.getCellWithHigherSINR();
                    c.addUserAssigned();
                    u.setServingCell(c);
                }

//            Primera propuesta (Op1): encender celdas menores del x% de usuarios con peores capacidades
//
//            double n = 0.1;
//
//            List<User> worst = new ArrayList<>(udn_.getUsers());
//            worst.sort(Comparator.comparing(o -> (o.capacityMIMO(udn_, o.getServingCell().getSharedBWForAssignedUsers()))));
//            worst = worst.subList(0, (int) (n * worst.size()));
//
//            for (User u : worst) {
//                CellType oldType = u.getServingCell().getType();
//                CellType newType;
//
//                switch (oldType) {
//                    case MACRO:
//                        newType = MICRO;
//                        break;
//                    case MICRO:
//                        newType = PICO;
//                        break;
//                    default:
//                        newType = FEMTO;
//                        break;
//                }
//
//                boolean changed = false;
//                do {
//                    Cell c = udn_.getClosestCellByType(udn_.getGridPoint(u.getX(), u.getY(), u.getZ()), newType);
//
//                    if (c.getBTS().getPoint().equals(u.getServingCell().getBTS().getPoint()) && c.getType() == oldType) {
//                        switch (newType) {
//                            case FEMTO:
//                                newType = PICO;
//                                break;
//                            case PICO:
//                                newType = MICRO;
//                                break;
//                            default:
//                                newType = FEMTO;
//                                break;
//                            //TODO uncomment if MACRO enabled
////                        case MICRO:
////                            newType = MACRO;
////                            break;
////                        case MACRO:
////                            break;
//                        }
//                    } else {
//                        c.setActivation(true);
//                        changed = true;
//                    }
//                } while (!changed);
//            }

                // Segunda propuoesta (Op2): encender celdas menores de los usuarios a más distancia de las celdas asignadas que la media

//            Map<User, Double> farthestMap = new HashMap<>();
//
//            for (User u : udn_.getUsers()) {
//                farthestMap.put(u, udn_.distance(u.getX(), u.getY(), u.getZ(), u.getServingCell().getBTS().getX(), u.getServingCell().getBTS().getY(), u.getServingCell().getBTS().getZ()));
//            }
//
//            farthestMap = farthestMap.entrySet().stream().sorted(Map.Entry.comparingByValue()).collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue, (e1, e2) -> e1, LinkedHashMap::new));
//
//            double average = farthestMap.values().stream().mapToDouble(Double::doubleValue).average().orElse(0);
//
//            List<User> farthest = new ArrayList<>();
//
//            for (User u : farthestMap.keySet()) {
//                if (farthestMap.get(u) > average)
//                    farthest.add(u);
//            }
//
//            // TODO To obtain the number of active cells by type
////            int mayor = 0;
////            int menor = 0;
////
////            for (User u : farthest.keySet()) {
////                    if (farthest.get(u) > average)
////                        mayor++;
////                    else
////                        menor++;
////            }
////
////            System.out.println(mayor + " " + menor);
//
//            // Number of active different types of cells
////            Map<CellType, Integer> activeCells = new HashMap<>();
////            for (CellType type : CellType.values()){
////                if (type != MACRO)  // TODO Delete if MACRO enabled
////                    activeCells.put(type, udn_.getActiveCellsByType(type));
////            }
//            // Fin TODO
//
//            for (User u : farthest) {
//                CellType oldType = u.getServingCell().getType();
//                CellType newType;
//
//                switch (oldType) {
//                    case MACRO:
//                        newType = MICRO;
//                        break;
//                    case MICRO:
//                        newType = PICO;
//                        break;
//                    default:
//                        newType = FEMTO;
//                        break;
//                }
//
//                boolean changed = false;
//                do {
//                    Cell c = udn_.getClosestCellByType(u, newType);
//
//                    if (!u.getServingCell().equals(c)) {
//                        c.setActivation(true);
//                        changed = true;
//                    } else {
//                        switch (newType) {
//                            case FEMTO:
//                                newType = PICO;
//                                break;
//                            case PICO:
//                                newType = MICRO;
//                                break;
//                            default:
//                                newType = FEMTO;
//                                break;
//                            //TODO uncomment if MACRO enabled
//                            //                        case MICRO:
//                            //                            newType = MACRO;
//                            //                            break;
//                            //                        case MACRO:
//                            //                            break;
//                        }
//                    }
//                } while (!changed);
//            }

                // Tercera propuoesta (Op3): encender celdas menores cercanas al usuario mediana de la distancia usuario-BTS de las celdas con número de usuarios asignados mayor a la media

//            Map<Cell, Integer> cellMap = new HashMap<>();
//
//            for (Double d : udn_.cells_.keySet()) {
//                for (Cell c : udn_.cells_.get(d)) {
//                    if (c.isActive()) {
//                        cellMap.put(c, c.getAssignedUsers());
//                    }
//                }
//            }
//
//            cellMap = cellMap.entrySet().stream().sorted(Map.Entry.comparingByValue()).collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue, (e1, e2) -> e1, LinkedHashMap::new));
//
//            double average = cellMap.values().stream().mapToDouble(Integer::intValue).average().orElse(0);
//
//            List<Cell> cells = new ArrayList<>();
//            for (Cell c : cellMap.keySet()) {
//                if (c.getAssignedUsers() > average)
//                    cells.add(c);
//            }
//
//            for (Cell c : cells) {
//                Map<Integer, List<User>> cMap = new HashMap<>();
//                Map<Integer, Integer> count = new HashMap<>();
//                for (int i = 1; i <= 4; i++) {
//                    count.put(i, 0);
//                    cMap.put(i, new ArrayList<>());
//                }
//                for (User u : udn_.getUsers()) {
//                    if (u.getServingCell().equals(c)) {
//                        if (u.getX() < c.getBTS().getX()) {
//                            if (u.getY() < c.getBTS().getY()) {
//                                List<User> tmp = cMap.get(1);
//                                tmp.add(u);
//                                cMap.put(1, tmp);
//                                count.put(1, count.get(1) + 1);
//                            } else {
//                                List<User> tmp = cMap.get(3);
//                                tmp.add(u);
//                                cMap.put(3, tmp);
//                                count.put(3, count.get(3) + 1);
//                            }
//                        } else {
//                            if (u.getY() < c.getBTS().getY()) {
//                                List<User> tmp = cMap.get(2);
//                                tmp.add(u);
//                                cMap.put(2, tmp);
//                                count.put(2, count.get(2) + 1);
//                            } else {
//                                List<User> tmp = cMap.get(4);
//                                tmp.add(u);
//                                cMap.put(4, tmp);
//                                count.put(4, count.get(4) + 1);
//                            }
//                        }
//                    }
//                }
//                int zone = Collections.max(count.entrySet(), Comparator.comparingInt(Map.Entry::getValue)).getKey();
//                Map<User, Double> distances = new HashMap<>();
//                for (User u : cMap.get(zone)) {
//                    distances.put(u, udn_.distance2D(u.getX(), u.getY(), u.getServingCell().getBTS().getX(), u.getServingCell().getBTS().getY()));
//                }
//
//                distances = distances.entrySet().stream().sorted(Map.Entry.comparingByValue()).collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue, (e1, e2) -> e1, LinkedHashMap::new));
//
//                User median = null;
//                int middle;
//                if (distances.size() % 2 == 1)
//                    middle = distances.size() / 2;
//                else
//                    middle = distances.size() / 2 - 1;
//
//                int countMiddle = 0;
//                for (User u : distances.keySet()) {
//                    if (countMiddle == middle) {
//                        median = u;
//                        break;
//                    }
//                    countMiddle++;
//                }
//
//                CellType newType = FEMTO;
//                switch (median.getServingCell().getType()) {
//                    case MACRO:
//                        newType = MICRO;
//                        break;
//                    case MICRO:
//                        newType = PICO;
//                        break;
//                }
//
//                udn_.getClosestCellByType(udn_.getGridPoint(median.getX(), median.getY(), median.getZ()), newType);
//            }

                // Cuarta propuesta: encender celdas de la misma BTS con mayor frencuencia de trabajo que la que tiene el usuario asignada y apagar la anterior si solo estaba él asignado

                for (User u : udn_.getUsers()) {
                    BTS b = u.getServingCell().getBTS();
                    Cell best = u.getServingCell();
                    Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                    double max_sinr = p.computeSINR(u.getServingCell());

                    for (Sector s : b.getSectors()) {
                        for (Cell c : s.getCells()) {
                            if (!c.equals(u.getServingCell())) {
                                if (c.getType() == FEMTO) {
                                    double sinr = p.computeSINR(c);
                                    if (sinr > max_sinr) {
                                        best = c;
                                        max_sinr = sinr;
                                    }
                                } else if (c.getType() == PICO) {
                                    if (u.getServingCell().getType() == PICO || u.getServingCell().getType() == MICRO || u.getServingCell().getType() == MACRO) {
                                        double sinr = p.computeSINR(c);
                                        if (sinr > max_sinr) {
                                            best = c;
                                            max_sinr = sinr;
                                        }
                                    }
                                } else if (c.getType() == MICRO) {
                                    if (u.getServingCell().getType() == MICRO || u.getServingCell().getType() == MACRO) {
                                        double sinr = p.computeSINR(c);
                                        if (sinr > max_sinr) {
                                            best = c;
                                            max_sinr = sinr;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    best.setActivation(true);

                    if (u.getServingCell().getAssignedUsers() == 1 && !u.getServingCell().equals(best)) {
                        u.getServingCell().setActivation(false);
                    }
                }

            }

            modifySolution(solution);
        }
//        noUsersOp(1, solution);
    }

    /**
     * Given a user attached to the macrocell, it can be assigned to other cell
     * in case the SINR received from the other is bigger than a certain
     * threshold value.
     *
     * @param points List of points
     */
    public void macro1Op(Map<Double, List<Point>> points) {
        double threshold = 2;
        double sinr_limit = 4;

        //get the macrocell
        double macro_f = 2000;
        Cell macro;
        macro = udn_.cells_.get(macro_f).get(0);
        List<User> macro_users = new ArrayList<>();

        //get the users assigned to the macrocell
        for (User u : this.udn_.getUsers()) {
            if (u.getServingCell() == macro) {
                Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                if (p.computeSINR(macro) < sinr_limit) {
                    macro_users.add(u);
                }
            }
        }
        //apply the operator for these users

        int count = 0;

        //System.out.println("There are "+macro_users.size()+" users attached to the macrocell");
        if (!macro_users.isEmpty()) {
            for (int i = 0; i < macro_users.size(); i++) {
                User u = macro_users.get(i);
                Point p_ = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                double sinr_macro = p_.computeSINR(macro);
                Cell other = p_.getCellWithHigherSINRButMacro();
                double sinr_other = p_.computeSINR(other);
                int nuser = i + 1;
                //System.out.println("User: "+nuser);
                //System.out.println("SINR_macro: "+sinr_macro);
                //System.out.println("SINR_alternative: "+sinr_other);
                //System.out.println("La alternativa es una celda "+ other.getType().toString() + " con SINR: "+ sinr_other);
                if (sinr_other > threshold) {
                    u.setServingCell(other);
                    count++;
                }
            }
        }
        System.out.println("The operator macro1 has been applied " + count + " times");
    }

    /**
     * Given a user attached to the macrocell, it can be assigned to other cell
     * in case a certain condition is fulfilled.
     *
     * @param points List of points
     */
    public void macro2Op(Map<Double, List<Point>> points) {
        double threshold = 6;
        double sinr_limit = 12;

        //get the macrocell
        double macro_f = 2000;
        Cell macro;
        macro = udn_.cells_.get(macro_f).get(0);
        List<User> macro_users = new ArrayList<>();

        //get the users assigned to the macrocell
        for (User u : this.udn_.getUsers()) {
            if (u.getServingCell() == macro) {
                Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                if (p.computeSINR(macro) < sinr_limit) {
                    macro_users.add(u);
                }
            }
        }
        //apply the operator for these users

        int count = 0;

        if (!macro_users.isEmpty()) {
            for (User u : macro_users) {
                Point p_ = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                double sinr_macro = p_.computeSINR(macro);
                Cell other = p_.getCellWithHigherSINRButMacro();
                double sinr_other = p_.computeSINR(other);
                //Here comes the condition
                if ((sinr_macro - sinr_other) < threshold) {
                    u.setServingCell(other);
                    count++;
                }
            }
        }

        System.out.println("The operator macro2 has been applied " + count + " times");
    }

    /**
     * If more than a certain amount of users are connected to a cell, one of
     * them will be switched to the next better one
     */
    public void tooManyUsersOp() {
        int count = 0;
        int threshold = 3; //calcular la media de todos y que haya como máximo 2 veces la media
        Cell alternative = null;
        Point user_location;
        Map<Double, Cell> bestCells;

        for (User u : this.udn_.getUsers()) {
            user_location = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
            if (u.getServingCell().getAssignedUsers() >= threshold) {

                bestCells = user_location.getCellsWithBestSINRs();
                //get the 2nd best cell
                int i = 1;
                for (Map.Entry<Double, Cell> actualEntry : bestCells.entrySet()) {
                    if (i == 2) {
                        alternative = actualEntry.getValue();
                        break;
                    } else {
                        i++;
                    }
                }
                u.setServingCell(alternative);
                count++;
            }
        }
        System.out.println("The operator tooManyUsers has been applied " + count + " times");
    }

    /**
     * Switch on those femtocells that can serve UEs.
     *
     * @param solution Solution
     */
    public void priorizeFemtoOp(double rate, BinarySolution solution) {
        if (new JavaRandomGenerator().nextDouble() < rate) {
            BitSet cso = solution.variables().get(0);

            //map the activation to the udn
            udn_.setCellActivation(cso);

            //recompute the signaling
            udn_.computeSignaling();

            //reset the UEs assigned to cells
            udn_.resetNumberOfUsersAssignedToCells();

            if (udn_.getTotalNumberOfActiveCells() > 0) {

                //Assign users to cells, to compute the BW allocated to them
                for (User u : this.udn_.getUsers()) {
                    Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());

                    Cell c = p.getCellWithHigherSINR();

                    c.addUserAssigned();

                    u.setServingCell(c);
                }

                //Look for the the candidate femtocells
                double threshold = 6; // 6 y 9 podrían valer: depende del tipo de celda origen: 6 dB por cada salto
                Cell alternative;
                Cell current;
                Point user_location;
                Map<Double, Cell> bestCells;

                for (User u : this.udn_.getUsers()) {
                    if ((u.getServingCell().getType() != FEMTO) || (u.getServingCell().getType() != PICO)) {
                        current = u.getServingCell();
                        user_location = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                        bestCells = user_location.getCellsWithBestSINRs();
                        for (Map.Entry<Double, Cell> actualEntry : bestCells.entrySet()) {
                            alternative = actualEntry.getValue();
                            if (user_location.computeSINR(alternative) > threshold) {
                                if ((alternative.getType() == FEMTO) || (alternative.getType() == PICO)) {
                                    u.setServingCell(alternative);
                                    alternative.addUserAssigned();
                                    current.removeUserAssigned();
                                    if (current.getAssignedUsers() == 0)
                                        current.setActivation(false);
                                    alternative.setActivation(true);
                                    break;
                                }
                            }
                        }
                    }//IF
                }//FOR

                //apply CSO -> switch off the remaining cells not serving any UE
                for (double frequency : this.udn_.cells_.keySet()) {
                    if (udn_.cells_.containsKey(frequency)) {
                        for (Cell c : udn_.cells_.get(frequency)) {
                            if (c.getAssignedUsers() == 0)
                                c.setActivation(false);
                        }
                    }
                }

                //Copy the modifications to the solution
                modifySolution(solution);
            }
        }
    }


    /**
     * Switch on those small cells (pico and femto) that can serve UEs.
     *
     * @param solution Solution
     */
    public void priorizeSmallCellsOp(double rate, BinarySolution solution) {
        if (new JavaRandomGenerator().nextDouble() < rate) {
            BitSet cso = solution.variables().get(0);

            //map the activation to the udn
            udn_.setCellActivation(cso);

            //recompute the signaling
            udn_.computeSignaling();

            //reset the UEs assigned to cells
            udn_.resetNumberOfUsersAssignedToCells();

            if (udn_.getTotalNumberOfActiveCells() > 0) {

                //Assign users to cells, to compute the BW allocated to them
                for (User u : this.udn_.getUsers()) {
                    Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());

                    Cell c = p.getCellWithHigherSINR();

                    c.addUserAssigned();

                    u.setServingCell(c);
                }

                //Look for the candidate femtocells
                double threshold = 1; //6 y 9 podrían valer: depende del tipo de celda origen: 6 dB por cada salto
                Cell alternative;
                Cell current;
                Point user_location;
                Map<Double, Cell> bestCells;

                for (User u : this.udn_.getUsers()) {
                    if ((u.getServingCell().getType() != FEMTO) || (u.getServingCell().getType() != PICO)) {
                        current = u.getServingCell();
                        user_location = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
                        bestCells = user_location.getCellsWithBestSINRs();
                        for (Map.Entry<Double, Cell> actualEntry : bestCells.entrySet()) {
                            alternative = actualEntry.getValue();
                            if (user_location.computeSINR(alternative) > threshold) {
                                if ((alternative.getType() == FEMTO) || (alternative.getType() == PICO)) {
                                    u.setServingCell(alternative);
                                    alternative.addUserAssigned();
                                    current.removeUserAssigned();
                                    if (current.getAssignedUsers() == 0)
                                        current.setActivation(false);
                                    alternative.setActivation(true);

                                    //recompute the signaling
                                    udn_.computeSignaling();

                                    //reset the UEs assigned to cells
                                    udn_.resetNumberOfUsersAssignedToCells();

                                    //Assign users to cells, to compute the BW allocated to them
                                    for (User us : this.udn_.getUsers()) {
                                        Point p = udn_.getGridPoint(us.getX(), us.getY(), us.getZ());

                                        Cell c = p.getCellWithHigherSINR();

                                        c.addUserAssigned();

                                        us.setServingCell(c);
                                    }
                                    break;
                                }
                            }
                        }
                    }//IF
                }//FOR

                //apply CSO -> switch off the remaining cells not serving any UE
                for (double frequency : this.udn_.cells_.keySet()) {
                    if (udn_.cells_.containsKey(frequency)) {
                        List<Cell> l = udn_.cells_.get(frequency);
                        for (Cell c : l) {
                            if (c.getAssignedUsers() == 0) {
                                c.setActivation(false);
                            }
                        }
                    }

                }

                //Copy the modifications to the solution
                modifySolution(solution);
            }
        }//if
    }


    /**
     * Turn off those BTSs that only have one active cell, saving the maintenance power
     *
     * @param rate     : application probability
     * @param solution : Solution to be modified by the operator
     */
    public void maintenancePowerOp(double rate, BinarySolution solution) {
        if (new JavaRandomGenerator().nextDouble() < rate) {
            BitSet cso = solution.variables().get(0);

            udn_.setCellActivation(cso);
            udn_.computeSignaling();
            udn_.resetNumberOfUsersAssignedToCells();

            if (udn_.getTotalNumberOfActiveCells() > 0) {
                // v1
                for (List<BTS> btss : udn_.btss_.values()) {
                    for (BTS bts : btss) {
                        if (bts.getNumberOfActiveCells() == 1) {
                            //Turn off the active cell
                            for (Sector sector : bts.getSectors()) {
                                for (Cell cell : sector.getCells()) {
                                    if (cell.isActive()) {
                                        cell.setActivation(false);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                // v2
                //            for (List<BTS> btss : udn_.btss_.values()) {
                //                for (BTS bts : btss) {
                //                    if (bts.getNumberOfActiveCells() == 1) {
                //                        //Turn off the active cell
                //                        for (Sector sector : bts.getSectors()) {
                //                            for (Cell cell : sector.getCells()) {
                //                                if (cell.isActive()) {
                //                                    if (cell.getAssignedUsers() > 0) {
                //                                        for (User u : udn_.getUsers()) {
                //                                            if (u.getServingCell().getID() == cell.getID()) {
                //                                                CellType newType;
                //                                                switch (cell.getType()) {
                //                                                    case MACRO:
                //                                                        newType = MICRO;
                //                                                        break;
                //                                                    case MICRO:
                //                                                        newType = PICO;
                //                                                        break;
                //                                                    default:
                //                                                        newType = FEMTO;
                //                                                }
                //                                                udn_.getClosestCellByType(udn_.getGridPoint(u.getX(), u.getY(), u.getZ()), newType).setActivation(true);
                //                                                break;
                //                                            }
                //                                        }
                //                                    }
                //                                    cell.setActivation(false);
                //                                    break;
                //                                }
                //                            }
                //                        }
                //                    }
                //                }
                //            }

                modifySolution(solution);
            }
        }//if
    }


//    
//    /**
//     * Given a user attached to the a macro or microcell, it can be assigned to an small cell (femto, pico)
//     * in case the SINR received from the other is bigger than a certain
//     * threshold value.
//     *
//     * @param rate
//     * @param solution
//     */
//    public void smallCellsOp(double rate, Solution solution) {
//        int count = 0;
//        if (PseudoRandom.randDouble() < rate) {
//            Binary cso = ((Binary) solution.getDecisionVariables()[0]);
//
//            //map the activation to the udn
//            udn_.setCellActivation(cso);
//
//            //recompute the signaling
//            udn_.computeSignaling();
//
//            //reset the UEs assigned to celss
//            udn_.resetNumberOfUsersAssignedToCells();
//
//            //Assign users to cells, to compute the BW allocated to them
//            for (User u : this.udn_.getUsers()) {
//                Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
//
//                Cell c = p.getCellWithHigherSINR();
//
//                c.addUserAssigned();
//
//                u.setServingCell(c);
//            }
//
//            double threshold = 1;
//
//            List<User> micro_users = new ArrayList<>();
//
//            //get the users assigned to a microcell
//            for (User u : this.udn_.getUsers()) {
//                Cell uCell = u.getServingCell();
//                if (uCell.getType() == CellType.MICRO) {
//                    Point p = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
//                    micro_users.add(u);
//                }
//            }
//            //apply the operator for these users
//
//            if (!micro_users.isEmpty()) {
//                for (int i = 0; i < micro_users.size(); i++) {
//                    User u = micro_users.get(i);
//                    Cell current = u.getServingCell();
//                    Point p_ = udn_.getGridPoint(u.getX(), u.getY(), u.getZ());
//                    Cell alternative = p_.getSmallCellWithHigherSINR();
//                    double sinr_alternative = p_.computeSINR(alternative);
//                    if (sinr_alternative > threshold) {
//                        u.setServingCell(alternative);
//                        alternative.addUserAssigned();
//                        current.removeUserAssigned();
//                        count++;
//                    }
//                }
//            }
//            //Copy the modifications to the solution
//            modifySolution(solution);
//        }//if
//        
//       // System.out.println("The operator smallCellOp has been applied " + count + " times");
//        
//    }

    /**
     * Activates/deactivates BTSs in the solution according to the information
     * enclosed in the modified network of the problem
     *
     * @param solution Solution
     */
    public void modifySolution(BinarySolution solution) {   // TODO comprobar que se modifica
        BitSet cso = solution.variables().get(0);
        int bts = 0;

        for (List<Cell> cells : udn_.cells_.values()) {
            for (Cell c : cells) {
                if (c.getType() != CellType.MACRO) {
                    cso.set(bts, c.isActive());
                    bts++;
                }
            }
        }
    }

    /**
     * Extract operators configuration from file
     *
     * @param configFile Config file
     */
    public void loadOperatorsConfig(String configFile) {
        //read operators configuration
        //double noUsersOp_rate, macro1Op_rate, macro2Op_rate, tooManyUsersOp_rate, priorizeFemtoOp_rate, maintenancePowerOp_rate;
        Properties pro = new Properties();
        this.operators_ = new HashMap<>();
        try {
            System.out.println("Loading operators configuration file...");
            pro.load(new FileInputStream(configFile));
            int numOperators = Integer.parseInt(pro.getProperty("numOperators", "2"));

            for (int i = 1; i <= numOperators; i++) {
                String operatorName = pro.getProperty("operator" + i, "unknownOp");
                double rate = Double.parseDouble(pro.getProperty("rate" + i, "0.0"));
                this.operators_.put(operatorName, rate);
            }

        } catch (IOException e) {
            System.out.println(e + "Error loading operators configuration: " + configFile);
            System.exit(-1);
        }


        System.out.println("OPERATORS APPLIED: ");
        for (String operator : this.operators_.keySet())
            System.out.println(operator + ", rate: " + this.operators_.get(operator).toString());
    }
} // CSO
