package org.uma.jmetal.algorithm.multiobjective.mocell;

import org.uma.jmetal.operator.crossover.CrossoverOperator;
import org.uma.jmetal.operator.mutation.MutationOperator;
import org.uma.jmetal.operator.selection.SelectionOperator;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.multiobjective.UDN.StaticCSO;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.util.archive.BoundedArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.legacy.front.Front;
import org.uma.jmetal.util.legacy.front.impl.ArrayFront;
import org.uma.jmetal.util.measure.Measurable;
import org.uma.jmetal.util.measure.MeasureManager;
import org.uma.jmetal.util.measure.impl.BasicMeasure;
import org.uma.jmetal.util.measure.impl.CountingMeasure;
import org.uma.jmetal.util.measure.impl.DurationMeasure;
import org.uma.jmetal.util.measure.impl.SimpleMeasureManager;
import org.uma.jmetal.util.neighborhood.Neighborhood;

import java.util.Comparator;
import java.util.List;

public class HybridMOCell<S extends Solution<?>> extends MOCell<S> {

    protected SimpleMeasureManager measureManager;
    protected BasicMeasure<List<S>> solutionListMeasure;
    protected CountingMeasure evaluations;
    protected DurationMeasure durationMeasure;

    protected Front referenceFront;

    public HybridMOCell(Problem<S> problem, int maxEvaluations, int populationSize, BoundedArchive<S> archive, Neighborhood<S> neighborhood, CrossoverOperator<S> crossoverOperator,
                        MutationOperator<S> mutationOperator, SelectionOperator<List<S>, S> selectionOperator, SolutionListEvaluator<S> evaluator) {
        super(problem, maxEvaluations, populationSize, archive, neighborhood, crossoverOperator, mutationOperator, selectionOperator, evaluator);

        referenceFront = new ArrayFront();
    }

    @Override
    protected List<S> evaluatePopulation(List<S> population) {
        for (S s : population)
            ((StaticCSO) problem).intelligentSwitchOff((BinarySolution) s);

        population = evaluator.evaluate(population, getProblem());

//        if (evaluations.get() != 0 && evaluations.get() % 100 == 0)
//            new SolutionListOutput(population).setFunFileOutputContext(new DefaultFileOutputContext("HybridMOCell." + evaluations.get().toString(), " ")).print();

        return population;
    }

    @Override
    public String getName() {
        return "HybridMOCell";
    }
}
