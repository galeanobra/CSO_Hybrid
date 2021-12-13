package org.uma.jmetal.algorithm.multiobjective.smsemoa;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.nsgaii.HybridNSGAII;
import org.uma.jmetal.algorithm.multiobjective.nsgaii.NSGAIIBuilder;
import org.uma.jmetal.example.AlgorithmRunner;
import org.uma.jmetal.operator.crossover.CrossoverOperator;
import org.uma.jmetal.operator.crossover.impl.TwoPointCrossover;
import org.uma.jmetal.operator.mutation.MutationOperator;
import org.uma.jmetal.operator.mutation.impl.BitFlipMutation;
import org.uma.jmetal.operator.selection.SelectionOperator;
import org.uma.jmetal.operator.selection.impl.BinaryTournamentSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.multiobjective.UDN.StaticCSO;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.util.AbstractAlgorithmRunner;
import org.uma.jmetal.util.chartcontainer.ChartContainer;
import org.uma.jmetal.util.errorchecking.JMetalException;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.measure.MeasureListener;
import org.uma.jmetal.util.measure.MeasureManager;
import org.uma.jmetal.util.measure.impl.BasicMeasure;
import org.uma.jmetal.util.measure.impl.CountingMeasure;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

public class SMSEMOA_CSO_main extends AbstractAlgorithmRunner {
    public static void main(String[] args) throws JMetalException, FileNotFoundException {
        Problem<BinarySolution> problem;
        Algorithm<List<BinarySolution>> algorithm;
        CrossoverOperator<BinarySolution> crossover;
        MutationOperator<BinarySolution> mutation;
        SelectionOperator<List<BinarySolution>, BinarySolution> selection;

        int popSize = Integer.parseInt(args[0]);    // Population size
        int numEvals = Integer.parseInt(args[1]);   // Stopping condition
        int run = Integer.parseInt(args[2]);        // Seed selection
        int taskID = Integer.parseInt(args[3]);     // Task ID (for filename)
        int jobID = Integer.parseInt(args[4]);      // Job ID (for filename)
        String name = args[5];                      // Name (for output directory)
        String main = args[6];                      // Main configuration file

        problem = new StaticCSO(main, run);

        double crossoverProbability = 0.9;
        double mutationProbability = 1.0 / ((StaticCSO) problem).getTotalNumberOfActivableCells();

        crossover = new TwoPointCrossover(crossoverProbability);
        mutation = new BitFlipMutation(mutationProbability);
        selection = new BinaryTournamentSelection<>();

        algorithm = new SMSEMOABuilder<>(problem, crossover, mutation).setPopulationSize(popSize).setSelectionOperator(selection).setMaxEvaluations(numEvals)
                .setVariant(SMSEMOABuilder.SMSEMOAVariant.HybridSMSEMOA).build();

        AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm).execute();

        List<BinarySolution> population = algorithm.getResult();

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

        System.out.println("Total execution time: " + algorithmRunner.getComputingTime() + "ms");
        System.out.println("Objectives values have been written to file " + FUN);
        System.out.println("Variables values have been written to file " + VAR);

//        PlotFront plot = new Plot3D(new ArrayFront(population).getMatrix(), problem.getName() + " (NSGA-II)");
//        plot.plot();
//        PlotFront plot = new PlotSmile(new ArrayFront(population).getMatrix(), problem.getName() + " (NSGA-II)");
//        plot.plot();
    }
}