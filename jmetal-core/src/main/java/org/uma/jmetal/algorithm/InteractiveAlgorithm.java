package org.uma.jmetal.algorithm;

import java.util.List;

public interface InteractiveAlgorithm<S,R> extends Algorithm<R>{
  void updatePointOfInterest(List<Double> newReferencePoints);
}
