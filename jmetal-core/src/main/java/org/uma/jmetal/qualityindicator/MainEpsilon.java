package org.uma.jmetal.qualityindicator;

import org.uma.jmetal.qualityindicator.impl.Epsilon;
import org.uma.jmetal.qualityindicator.impl.hypervolume.impl.PISAHypervolume;
import org.uma.jmetal.util.NormalizeUtils;
import org.uma.jmetal.util.VectorUtils;

import java.io.IOException;

public class MainEpsilon {
    public static void main(String[] args) {
        if (args.length < 2) {
            System.err.println("Epsilon::Main: Usage: java MainEpsilon <FrontFile> <TrueFrontFile>");
            System.exit(1);
        }

        double[][] front = new double[0][];
        double[][] referenceFront = new double[0][];

        try {
            front = VectorUtils.readVectors(args[0]);
            referenceFront = VectorUtils.readVectors(args[1]);
        } catch (IOException e) {
            e.printStackTrace();
        }

        double[][] normalizedReferenceFront = NormalizeUtils.normalize(referenceFront);
        double[][] normalizedFront = NormalizeUtils.normalize(
                front,
                NormalizeUtils.getMinValuesOfTheColumnsOfAMatrix(referenceFront),
                NormalizeUtils.getMaxValuesOfTheColumnsOfAMatrix(referenceFront));

        System.out.println(new Epsilon(normalizedReferenceFront).compute(normalizedFront));
    }
}
