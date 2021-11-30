package org.uma.jmetal.problem.multiobjective.UDN.model.cells;

import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;

/**
 * @author paco
 */
public class MicroCell extends Cell {

    public MicroCell(
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
            double workingFrequency) {
        super(udn, sector,
                name,
                x, y, z,
                transmittedPower,
                alfa,
                beta,
                delta,
                transmitterGain,
                receptorGain,
                workingFrequency);

        this.type_ = UDN.CellType.MICRO;
        this.cost_ = 500;
        this.active_ = false;
        //MIMO Capacity Parameters (precomputed)
        this.singularValuesH = new double[]{2.9464, 2.0136};
        this.numAntennasRx = 2;
        this.numAntennasTx = 8;
    }

    public MicroCell(Cell c) {
        this.id_ = c.id_;
        this.sector_ = new Sector(c.sector_);
    }

    @Override
    Cell newInstance() {
        return new MicroCell(this);
    }
}
