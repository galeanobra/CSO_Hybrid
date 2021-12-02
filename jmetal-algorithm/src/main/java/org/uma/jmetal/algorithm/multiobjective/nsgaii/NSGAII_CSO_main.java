package org.uma.jmetal.algorithm.multiobjective.nsgaii;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.example.AlgorithmRunner;
import org.uma.jmetal.example.multiobjective.nsgaii.NSGAIIMeasuresWithChartsRunner;
import org.uma.jmetal.example.multiobjective.nsgaii.RNSGAIIWithChartsRunner;
import org.uma.jmetal.lab.visualization.plot.PlotFront;
import org.uma.jmetal.lab.visualization.plot.impl.PlotSmile;
import org.uma.jmetal.operator.crossover.CrossoverOperator;
import org.uma.jmetal.operator.crossover.impl.SinglePointCrossover;
import org.uma.jmetal.operator.crossover.impl.TwoPointCrossover;
import org.uma.jmetal.operator.mutation.MutationOperator;
import org.uma.jmetal.operator.mutation.impl.BitFlipMutation;
import org.uma.jmetal.operator.selection.SelectionOperator;
import org.uma.jmetal.operator.selection.impl.BinaryTournamentSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.multiobjective.UDN.StaticCSO;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.solution.doublesolution.DoubleSolution;
import org.uma.jmetal.util.AbstractAlgorithmRunner;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.chartcontainer.ChartContainer;
import org.uma.jmetal.util.chartcontainer.ChartContainerWithReferencePoints;
import org.uma.jmetal.util.errorchecking.JMetalException;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.legacy.front.impl.ArrayFront;
import org.uma.jmetal.util.measure.MeasureListener;
import org.uma.jmetal.util.measure.MeasureManager;
import org.uma.jmetal.util.measure.impl.BasicMeasure;
import org.uma.jmetal.util.measure.impl.CountingMeasure;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

public class NSGAII_CSO_main extends AbstractAlgorithmRunner {
    public static void main(String[] args) throws JMetalException, FileNotFoundException {
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
        boolean display = Boolean.parseBoolean(args[7]);

        problem = new StaticCSO(main, run);

        double crossoverProbability = 0.9;
        crossover = new SinglePointCrossover(crossoverProbability);

        double mutationProbability = 1.0 / ((StaticCSO) problem).getTotalNumberOfActivableCells();
        mutation = new BitFlipMutation(mutationProbability);

        selection = new BinaryTournamentSelection<>();

        algorithm = new NSGAIIBuilder<>(problem, crossover, mutation, popSize).setSelectionOperator(selection).setMaxEvaluations(numEvals)
                .setVariant(NSGAIIBuilder.NSGAIIVariant.HybridNSGAII).build();

        if (display) {
            MeasureManager measureManager = ((HybridNSGAII<BinarySolution>) algorithm).getMeasureManager();
            BasicMeasure<List<BinarySolution>> solutionListMeasure = (BasicMeasure<List<BinarySolution>>) measureManager.<List<BinarySolution>>getPushMeasure("currentPopulation");
            CountingMeasure iterationMeasure = (CountingMeasure) measureManager.<Long>getPushMeasure("currentEvaluation");

            ChartContainer chart = new ChartContainer(algorithm.getName(), 100);
            chart.setFrontChart(0, 1, "resources/referenceFrontsCSV/0.csv");
//        chart.setReferencePoint(convertReferencePointListToListOfLists(referencePoint, problem.getNumberOfObjectives()));
            chart.initChart();

            solutionListMeasure.register(new NSGAII_CSO_main.ChartListener(chart));
            iterationMeasure.register(new NSGAII_CSO_main.IterationListener(chart));
        }


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
//        PlotFront plot = new PlotSmile(new ArrayFront(population).getMatrix(), problem.getName() + " (NSGA-II)");
//        plot.plot();
    }

    private static class ChartListener implements MeasureListener<List<BinarySolution>> {
        private final ChartContainer chart;
        private int iteration = 0;

        public ChartListener(ChartContainer chart) {
            this.chart = chart;
            this.chart.getFrontChart().setTitle("Evaluation: " + this.iteration);
        }

        private void refreshChart(List solutionList) {
            if (this.chart != null) {
                iteration++;
                this.chart.getFrontChart().setTitle("Iteration: " + this.iteration);
                this.chart.updateFrontCharts(solutionList);
                this.chart.refreshCharts();
            }
        }

        @Override
        synchronized public void measureGenerated(List solutions) {
            refreshChart(solutions);
        }
    }

    private static class IterationListener implements MeasureListener<Long> {
        ChartContainer chart;

        public IterationListener(ChartContainer chart) {
            this.chart = chart;
            this.chart.getFrontChart().setTitle("Iteration: " + 0);
        }

        @Override
        synchronized public void measureGenerated(Long iteration) {
            if (this.chart != null) {
                this.chart.getFrontChart().setTitle("Iteration: " + iteration);
            }
        }
    }

    private static List<List<Double>> convertReferencePointListToListOfLists(List<Double> referencePoints, int numberOfObjectives) {
        List<List<Double>> referencePointList;
        referencePointList = new ArrayList<>();

        for (int i = 0; i <= (referencePoints.size() - numberOfObjectives); i += numberOfObjectives) {
            List<Double> newReferencePoint = new ArrayList<>(numberOfObjectives);
            for (int j = i; j < (i + numberOfObjectives); j++) {
                newReferencePoint.add(referencePoints.get(j));
            }

            referencePointList.add(newReferencePoint);
        }

        return referencePointList;
    }
}