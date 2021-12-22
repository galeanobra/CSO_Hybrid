package org.uma.jmetal.algorithm.multiobjective.moead;

import org.uma.jmetal.algorithm.multiobjective.moead.util.MOEADUtils;
import org.uma.jmetal.operator.crossover.CrossoverOperator;
import org.uma.jmetal.operator.crossover.impl.DifferentialEvolutionCrossover;
import org.uma.jmetal.operator.mutation.MutationOperator;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.multiobjective.UDN.StaticCSO;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.solution.doublesolution.DoubleSolution;

import java.util.ArrayList;
import java.util.List;

public class HybridMOEAD<S extends Solution<?>> extends AbstractMOEAD<S> {
    public HybridMOEAD(Problem<S> problem,
                       int populationSize,
                       int resultPopulationSize,
                       int maxEvaluations,
                       MutationOperator<S> mutation,
                       CrossoverOperator<S> crossover,
                       FunctionType functionType,
                       String dataDirectory,
                       double neighborhoodSelectionProbability,
                       int maximumNumberOfReplacedSolutions,
                       int neighborSize) {
        super(problem, populationSize, resultPopulationSize, maxEvaluations, crossover, mutation, functionType, dataDirectory, neighborhoodSelectionProbability, maximumNumberOfReplacedSolutions, neighborSize);
    }

    public HybridMOEAD(Problem<S> problem, int populationSize, int resultPopulationSize, int maxEvaluations, CrossoverOperator<S> crossoverOperator, MutationOperator<S> mutation, FunctionType functionType, String dataDirectory, double neighborhoodSelectionProbability, int maximumNumberOfReplacedSolutions, int neighborSize) {
        super(problem, populationSize, resultPopulationSize, maxEvaluations, crossoverOperator, mutation, functionType, dataDirectory, neighborhoodSelectionProbability, maximumNumberOfReplacedSolutions, neighborSize);
    }

    @Override
    public void run() {
        initializePopulation();
        initializeUniformWeight();
        initializeNeighborhood();
        idealPoint.update(population);

        evaluations = populationSize;
        do {
            int[] permutation = new int[populationSize];
            MOEADUtils.randomPermutation(permutation, populationSize);

            for (int i = 0; i < populationSize; i++) {
                int subProblemId = permutation[i];
                NeighborType neighborType = chooseNeighborType();
                List<S> parents = parentSelection(subProblemId, neighborType);

                List<S> children = crossoverOperator.execute(parents);

                S child = children.get(0);
                mutationOperator.execute(child);

                ((StaticCSO) problem).intelligentSwitchOff((BinarySolution) child);

                problem.evaluate(child);
                evaluations++;

                idealPoint.update(child.objectives());
                updateNeighborhood(child, subProblemId, neighborType);
            }
        } while (evaluations < maxEvaluations);

    }

    protected void initializePopulation() {
        population = new ArrayList<>(populationSize);
        for (int i = 0; i < populationSize; i++) {
            S newSolution = problem.createSolution();

            problem.evaluate(newSolution);
            population.add(newSolution);
        }
    }

    @Override
    public String getName() {
        return "HybridMOEAD";
    }

    @Override
    public String getDescription() {
        return "Hybrid Multi-Objective Evolutionary Algorithm based on Decomposition";
    }
}
