//  NSGAII_main.java
//
//  Author:
//       Antonio J. Nebro <antonio@lcc.uma.es>
//       Juan J. Durillo <durillo@lcc.uma.es>
//
//  Copyright (c) 2011 Antonio J. Nebro, Juan J. Durillo
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
// 
//  You should have received a copy of the GNU Lesser General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
package org.uma.jmetal.algorithm.multiobjective.sparseea;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.operator.Operator;
import org.uma.jmetal.operator.selection.impl.BinaryTournamentSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.multiobjective.UDN.StaticCSO;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;

import java.util.HashMap;
import java.util.List;

/**
 * Class to configure and execute the SparseEA algorithm.
 */
public class SparseEA_CSO_main {

    /**
     * @param args Command line arguments.
     */
    public static void main(String[] args) throws SecurityException, ClassNotFoundException {
        Problem problem;        // The problem to solve
        HybridSparseEA<BinarySolution> algorithm;    // The algorithm to use
        Operator crossover;     // Crossover operator
        Operator mutation;      // Mutation operator
        Operator selection;     // Selection operator

        HashMap<String, Object> parameters; // Operator parameters

        int popSize = Integer.parseInt(args[0]);    // Population size
        int numEvals = Integer.parseInt(args[1]);   // Max num of evaluations
//        int tabuSize = Integer.parseInt(args[2]);   // Tabu list size
        int run = Integer.parseInt(args[2]);        // Run ID (for pseudorandom)
        int taskID = Integer.parseInt(args[3]);     // Slurm task ID (for filename)
        int jobID = Integer.parseInt(args[4]);      // Slurm jobID ID (for filename)
        String name = args[5];                      // Name (for output directory)
        String main = args[6];                      // Main config file

        problem = new StaticCSO(main, run);

        double crossoverProbability = 0.5; //0.9;
        double mutationProbability = 0.5; //1.0 / ((StaticCSO) problem).getTotalNumberOfActivableCells();

        algorithm = new HybridSparseEA(problem, numEvals, popSize, crossoverProbability, mutationProbability, new BinaryTournamentSelection<>(), ((StaticCSO) problem).getTotalNumberOfActivableCells());

        // Execute the Algorithm
        long initTime = System.currentTimeMillis();
        List<BinarySolution> population = algorithm.execute();
        long estimatedTime = System.currentTimeMillis() - initTime;

        // Set the output directory according to the system (config folder if Condor or Windows, out folder if Picasso or UNIX system)
//        String FUN = System.getProperty("os.name").toLowerCase().contains("win") ? name + ".FUN." + taskID + "." + jobID : "out/" + name + "/FUN/" + name + ".FUN." + taskID + "." + jobID;
//        String VAR = System.getProperty("os.name").toLowerCase().contains("win") ? name + ".VAR." + taskID + "." + jobID : "out/" + name + "/VAR/" + name + ".VAR." + taskID + "." + jobID;
        // For local debug, comment previous lines and uncomment these
        String FUN = name + ".FUN." + taskID + "." + jobID;
        String VAR = name + ".VAR." + taskID + "." + jobID;

        new SolutionListOutput(population)
                .setVarFileOutputContext(new DefaultFileOutputContext(VAR, " "))
                .setFunFileOutputContext(new DefaultFileOutputContext(FUN, " "))
                .print();

        System.out.println("Total execution time: " + estimatedTime + "ms");
        System.out.println("Objectives values have been written to file " + FUN);
        System.out.println("Variables values have been written to file " + VAR);
    }
}
