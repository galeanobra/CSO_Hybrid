package org.uma.jmetal.problem.multiobjective.UDN.model.cells;

import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;

/**
 * @author paco
 */
public class FemtoCell extends Cell {

    public FemtoCell(
            UDN udn,
            Sector sector,
            String name,
            int x, int y, int z,
            double transmittedPower,
            double alfa,
            double beta,
            double delta,
            double transmitterGain,
            double receptorGain,
            double workingFrequency,
            double coverageRadius) {

        super(udn, sector,
                name,
                x, y, z,
                transmittedPower,
                alfa,
                beta,
                delta,
                transmitterGain,
                receptorGain,
                workingFrequency
        );

        this.type_ = UDN.CellType.FEMTO;
        this.sector_.coverageRadius_ = coverageRadius;
        this.cost_ = 100;
        this.active_ = false;

        //MIMO Capacity Parameters (precomputed)
        this.singularValuesH = new double[]{18.0985, 7.4748, 16.3379, 15.7663, 15.5863, 14.8828, 14.4565, 13.9116};
        this.numAntennasRx = 8;
        this.numAntennasTx = 256;
    }

    public FemtoCell(Cell c) {
        this.id_ = c.id_;
        this.sector_ = new Sector(c.sector_);
    }

    public FemtoCell() {
        super();
    }

    @Override
    Cell newInstance() {
        return new FemtoCell(this);
    }
}
