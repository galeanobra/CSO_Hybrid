package org.uma.jmetal.algorithm.multiobjective.nsgaii;

import org.uma.jmetal.operator.crossover.CrossoverOperator;
import org.uma.jmetal.operator.mutation.MutationOperator;
import org.uma.jmetal.operator.selection.SelectionOperator;
import org.uma.jmetal.operator.selection.impl.RankingAndCrowdingSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.multiobjective.UDN.StaticCSO;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.util.SolutionListUtils;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class HybridNSGAII<S extends Solution<?>> extends NSGAII<S> {

    public HybridNSGAII(Problem<S> problem, int maxEvaluations, int populationSize, int matingPoolSize, int offspringPopulationSize, CrossoverOperator<S> crossoverOperator,
                        MutationOperator<S> mutationOperator, SelectionOperator<List<S>, S> selectionOperator, Comparator<S> dominanceComparator,
                        SolutionListEvaluator<S> evaluator) {
        super(problem, maxEvaluations, populationSize, matingPoolSize, offspringPopulationSize, crossoverOperator, mutationOperator, selectionOperator,
                new DominanceComparator<S>(), evaluator);
    }

    @Override
    protected List<S> evaluatePopulation(List<S> population) {
        for (S s : population) {
            ((StaticCSO) problem).intelligentSwitchOff((BinarySolution) s);
        }

        population = evaluator.evaluate(population, getProblem());

        return population;
    }
}
