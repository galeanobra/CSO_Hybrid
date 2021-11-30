package org.uma.jmetal.algorithm.multiobjective.nsgaii;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.example.AlgorithmRunner;
import org.uma.jmetal.lab.visualization.plot.PlotFront;
import org.uma.jmetal.lab.visualization.plot.impl.PlotSmile;
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
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.errorchecking.JMetalException;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.legacy.front.impl.ArrayFront;

import java.util.List;

public class NSGAII_CSO_main extends AbstractAlgorithmRunner {
    public static void main(String[] args) throws JMetalException {
        Problem<BinarySolution> problem;
        Algorithm<List<BinarySolution>> algorithm;
        CrossoverOperator<BinarySolution> crossover;
        MutationOperator<BinarySolution> mutation;
        SelectionOperator<List<BinarySolution>, BinarySolution> selection;

        int popSize = Integer.parseInt(args[0]);
        int numEvals = Integer.parseInt(args[1]);
        int run = Integer.parseInt(args[2]);
        int taskID = Integer.parseInt(args[3]);     // Task ID (for filename)
        int jobID = Integer.parseInt(args[4]);      // Job ID (for filename)
        String name = args[5];                      // Name (for output directory)
        String main = args[6];                      // Main configuration file

        problem = new StaticCSO(main, run);

        double crossoverProbability = 0.9;
        crossover = new TwoPointCrossover(crossoverProbability);

        double mutationProbability = 1.0 / ((StaticCSO) problem).getTotalNumberOfActivableCells();
        mutation = new BitFlipMutation(mutationProbability);

        selection = new BinaryTournamentSelection<>();

        algorithm = new NSGAIIBuilder<BinarySolution>(problem, crossover, mutation, popSize).setSelectionOperator(selection).setMaxEvaluations(numEvals)
                .setVariant(NSGAIIBuilder.NSGAIIVariant.HybridNSGAII).build();

        AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm).execute();

        List<BinarySolution> population = algorithm.getResult();
        long computingTime = algorithmRunner.getComputingTime();

        new SolutionListOutput(population)
                .setVarFileOutputContext(new DefaultFileOutputContext("VAR.csv", ","))
                .setFunFileOutputContext(new DefaultFileOutputContext("FUN.csv", ","))
                .print();

        JMetalLogger.logger.info("Total execution time: " + computingTime + "ms");
        JMetalLogger.logger.info("Objectives values have been written to file FUN.csv");
        JMetalLogger.logger.info("Variables values have been written to file VAR.csv");

//        PlotFront plot = new Plot3D(new ArrayFront(population).getMatrix(), problem.getName() + " (NSGA-II)");
//        plot.plot();
        PlotFront plot = new PlotSmile(new ArrayFront(population).getMatrix(), problem.getName() + " (NSGA-II)");
        plot.plot();
    }
}