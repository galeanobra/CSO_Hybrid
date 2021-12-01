package org.uma.jmetal.solution.binarysolution;

import org.uma.jmetal.problem.multiobjective.UDN.model.cells.Cell;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.binarySet.BinarySet;

import java.util.List;

/**
 * Interface representing binary (bitset) solutions
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public interface BinarySolution extends Solution<BinarySet> {
    int getNumberOfBits(int index);

    int getTotalNumberOfBits();

    void setUEsToCellAssignment(List<Cell> assignment);

    List<Cell> getCurrentUesToCellAssignment();

    List<Cell> getPreviousUesToCellAssignment();

    void forgetUEsToCellAssignment();
}
