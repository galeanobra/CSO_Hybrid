package org.uma.jmetal.algorithm.multiobjective.sparseea;

import org.uma.jmetal.operator.selection.SelectionOperator;
import org.uma.jmetal.operator.selection.impl.RankingAndCrowdingSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.solution.binarysolution.BinarySolution;
import org.uma.jmetal.solution.binarysolution.impl.DefaultBinarySolution;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.densityestimator.impl.CrowdingDistanceDensityEstimator;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;
import org.uma.jmetal.util.pseudorandom.RandomGenerator;
import org.uma.jmetal.util.ranking.Ranking;
import org.uma.jmetal.util.ranking.impl.FastNonDominatedSortRanking;

import java.util.*;

/**
 * Implementation of SparseEA.
 * Y. Tian, X. Zhang, C. Wang and Y. Jin, "An Evolutionary Algorithm
 * for Large-Scale Sparse Multiobjective Optimization Problems," in
 * IEEE Transactions on Evolutionary Computation, vol. 24, no. 2,
 * pp. 380-393, April 2020.
 */
public class SparseEA<S extends Solution<?>> {

    //    private final List<Solution> tabu;
//    private int tabuSize;
    protected Problem<S> problem;
    protected int maxEvaluations;
    protected int populationSize;
    protected double crossoverProb;
    protected double mutationProb;
    protected SelectionOperator<List<S>, S> selectionOperator;
    protected int numberOfBits;

    protected int evaluations;

    private RandomGenerator<Double> randomGenerator;

    protected Comparator<S> dominanceComparator;

    /**
     * Constructor
     *
     * @param problem Problem to solve
     */
    public SparseEA(Problem<S> problem, int maxEvalutaions, int populationSize, float crossoverProb, float mutationProb, SelectionOperator<List<S>, S> selectionOperator, int numberOfBits) {
//        tabu = new ArrayList<>();
//        tabuSize = 5;
        this.problem = problem;
        this.maxEvaluations = maxEvalutaions;
        this.populationSize = populationSize;
        this.crossoverProb = crossoverProb;
        this.mutationProb = mutationProb;
        this.selectionOperator = selectionOperator;
        this.numberOfBits = numberOfBits;

        this.evaluations = 0;

        this.dominanceComparator = new DominanceComparator<S>();
    }

    /**
     * Runs the SparseEA algorithm.
     *
     * @return a <code>SolutionSet</code> that is a set of non dominated
     * solutions as a result of the algorithm execution
     */
    public List<BinarySolution> execute() {
        int populationSize;
        int maxEvaluations;
        double mutationProb;
        double crossoverProb;

        List<BinarySolution> population;
        List<BinarySolution> offspringPopulation;
        List<BinarySolution> union;

        int[] score;                        // Score of the decision variables
        int d;                              // Number of decision variables
        boolean[][] dec;                    // Denote the decision variables TODO double if Real optimization
        boolean[][] mask;                   // Denote the mask for decs

        SelectionOperator selectionOperator;

        CrowdingDistanceDensityEstimator distance = new CrowdingDistanceDensityEstimator();

//        tabuSize = (Integer) getInputParameter("tabuSize");

        // Initialize the variables
        d = this.numberOfBits;
//        dec = new boolean[d][d];
        mask = new boolean[d][d];   // Eye matrix

        boolean[][] variables = new boolean[d][d];  // dec x mask

        for (int i = 0; i < d; i++) {
            for (int j = 0; j < d; j++) {
                // Fill dec matrix
//                dec[i][j] = true;

                // Fill mask matrix and product matrix
                if (i == j) {
                    mask[i][j] = true;
                    variables[i][j] = true;    // dec[i][j] * mask[i][j] = dec[i][j] = true (1)
                } else {
                    mask[i][j] = false;
                    variables[i][j] = false;        // dec[i][j] * mask[i][j] = false (0)
                }
            }
        }

        // A population whose i-th solution is generated by de i-th rows of dec and mask (i-th row of variables)
        List<S> q = (List<S>) generatePopulation(d, d, variables);

        // Return non-dominated fronts
        FastNonDominatedSortRanking<S> ranking = (FastNonDominatedSortRanking<S>) new FastNonDominatedSortRanking<>(this.dominanceComparator).compute(q);
        score = new int[d];

        // The score is the rank of the subfront
        for (int i = 0; i < d; i++) {
            for (int j = 0; j < ranking.getNumberOfSubFronts(); j++) {
                if (ranking.getSubFront(j).contains(q.get(i))) {
                    score[i] = j;
                    break;
                }
            }
        }

//        dec = new boolean[populationSize][d];
//        for (boolean[] booleans : dec) {
//            Arrays.fill(booleans, true);
//        }

        mask = new boolean[this.populationSize][d];
        for (boolean[] booleans : mask)
            Arrays.fill(booleans, false);

        for (int i = 0; i < this.populationSize; i++) {
            for (int j = 0; j < JMetalRandom.getInstance().nextDouble() * d; j++) {
                int m = JMetalRandom.getInstance().nextInt(0, d - 1);
                int n = JMetalRandom.getInstance().nextInt(0, d - 1);

                if (score[m] < score[n])
                    mask[i][m] = true;
                else
                    mask[i][n] = true;
            }
        }

        // A population whose i-th solution is generated by de i-th rows of dec and mask
        population = generatePopulation(this.populationSize, d, variables);

        ranking = (FastNonDominatedSortRanking<S>) new FastNonDominatedSortRanking<>(this.dominanceComparator).compute(q);

        distance.compute(population);

        // Generations
        while (this.evaluations < this.maxEvaluations) {
            List<BinarySolution> _population = new ArrayList<>(2 * population.size());   // P' -> 2 * N population
            for (int i = 0; i < this.populationSize; i++)                              // Fill _population with 2N parents from population
                _population.add((BinarySolution) this.selectionOperator.execute((List<S>) population));

            // Union
            Set<BinarySolution> set = new HashSet<>();
            set.add((BinarySolution) population);
            set.add((BinarySolution) variation(_population, population, score, mask));

            List<BinarySolution> solutions = new ArrayList<>(set);
            population.clear();

            // Remove duplicates solutions
            for (BinarySolution solution : solutions) {
                if (!population.contains(solution))
                    population.add(solution);
            }

            RankingAndCrowdingSelection<S> rankingAndCrowdingSelection = new RankingAndCrowdingSelection<S>(this.populationSize, dominanceComparator);
            population = (List<BinarySolution>) rankingAndCrowdingSelection.execute((List<S>) population);
        }

        for (BinarySolution binarySolution : population) problem.evaluate((S) binarySolution);

        evaluations += population.size();

        return population;
    }

    /**
     * Generate a population whose i-th solution is generated by de i-th rows of dec and mask (i-th row of variables)
     *
     * @param populationSize Population size
     * @param d              Number of decision variables
     * @param variables      dec * mask
     * @return New population
     */
    private List<BinarySolution> generatePopulation(int populationSize, int d, boolean[][] variables) {
        List<BinarySolution> population = new ArrayList<>(populationSize);

        for (int i = 0; i < populationSize; i++) {
            BinarySolution s = new DefaultBinarySolution(Collections.singletonList(this.numberOfBits), problem.getNumberOfObjectives());
            BitSet bits = s.variables().get(0);
            for (int j = 0; j < d; j++)
                bits.set(j, variables[i][j]);


            problem.evaluate((S) s);
//            problem.evaluateConstraints(s);
            population.add(s);
        }

        evaluations += populationSize;

        return population;
    }

    /**
     * Variation of population P'. Algorithm 3 from the paper.
     *
     * @param _population 2 * N parents population
     * @param population  Population
     * @param score       Decision variables scores
     * @param mask        Mask matrix
     * @return Variation of population P' (_population)
     */
    public List<BinarySolution> variation(List<BinarySolution> _population, List<BinarySolution> population, int[] score, boolean[][] mask) {
        List<BinarySolution> _population_variation = new ArrayList<>(population.size()); // Population variation

        int attempts = 0;
        while (!_population.isEmpty()) {
            // Randomly select two parents from _population
            BinarySolution p = _population.get(JMetalRandom.getInstance().nextInt(0, _population.size() / 2 - 1));
            BinarySolution q = _population.get(JMetalRandom.getInstance().nextInt(_population.size() / 2, _population.size() - 1));

            int p_index = population.indexOf(p);
            int q_index = population.indexOf(q);

            boolean[] o_mask = mask[p_index];           // Offspring mask = parent[0] mask
            boolean[] _o_mask = invertArray(o_mask);    // Reverse o_mask

            // Crossover
            if (JMetalRandom.getInstance().nextDouble() < crossoverProb) {
                int[] index = getIndexCrossover(mask[p_index], invertArray(mask[q_index]));

                if (score[index[0]] > score[index[1]])
                    o_mask[index[0]] = false;
                else
                    o_mask[index[1]] = false;
            } else {
                int[] index = getIndexCrossover(invertArray(mask[p_index]), mask[q_index]);

                if (score[index[0]] < score[index[1]])
                    o_mask[index[0]] = true;
                else
                    o_mask[index[1]] = true;
            }

            // Mutation
            if (JMetalRandom.getInstance().nextDouble() < mutationProb) {
                int[] index = randomFromNonZero(o_mask);

                if (score[index[0]] > score[index[1]])
                    o_mask[index[0]] = false;
                else
                    o_mask[index[1]] = false;
            } else {
                int[] index = randomFromNonZero(_o_mask);

                if (score[index[0]] < score[index[1]])
                    o_mask[index[0]] = true;
                else
                    o_mask[index[1]] = true;
            }

            // Generate the dec of offspring
//                boolean[] offspring_dec = new boolean[d];
//                Arrays.fill(offspring_dec, true);

            BinarySolution s = new DefaultBinarySolution(Collections.singletonList(this.numberOfBits), problem.getNumberOfObjectives());
            BitSet bits = s.variables().get(0);
            for (int i = 0; i < o_mask.length; i++)
                bits.set(i, o_mask[i]);

            _population.remove(p);
            _population.remove(q);

            problem.evaluate((S) s);
//            problem_.evaluateConstraints(newSolution);
            _population_variation.add(s);

            this.evaluations++;

//            if (!tabu.contains(newSolution) || attempts > 5) {
//                attempts = 0;
//
//                // Remove parents from _population
//                _population.remove(_population.getSolutions().indexOf(p));
//                _population.remove(_population.getSolutions().indexOf(q));
//
//                problem_.evaluate(newSolution);
//                problem_.evaluateConstraints(newSolution);
//                _population_variation.add(newSolution);
//
//                evaluations += 1;
//            } else
//                attempts += 1;

//            if (tabuSize > 0 && !tabu.contains(newSolution)) {
//                if (tabu.size() == tabuSize)
//                    tabu.remove(0);
//                tabu.add(newSolution);
//            }
        }

        return _population_variation;
    }

    /**
     * Generate the reverse of a boolean array.
     *
     * @param array Boolean array
     * @return Reverse of the array
     */
    public boolean[] invertArray(boolean[] array) {
        for (int i = 0; i < array.length; i++) {
            array[i] = !array[i];
        }

        return array;
    }

    /**
     * Return a random index of the array that contains 1.
     *
     * @param array Boolean array
     * @return
     */
    public int[] randomFromNonZero(boolean[] array) {
        int m = 0, n = 0;
        do {
            if (m == 0 || !array[m])
                m = JMetalRandom.getInstance().nextInt(0, array.length - 1);
            if (n == 0 || !array[n])
                n = JMetalRandom.getInstance().nextInt(0, array.length - 1);
        } while (!array[m] && !array[n]);

        return new int[]{m, n};
    }

    /**
     * Generate the intersection between two boolean arrays and returns index for crossover.
     *
     * @param array1 First boolean array
     * @param array2 Second boolean array
     * @return Index for crossover
     */
    public int[] getIndexCrossover(boolean[] array1, boolean[] array2) {
        boolean[] inter = new boolean[array1.length];
        for (int i = 0; i < inter.length; i++) {
            inter[i] = array1[i] || array2[i];
        }

        return randomFromNonZero(inter);
    }
}