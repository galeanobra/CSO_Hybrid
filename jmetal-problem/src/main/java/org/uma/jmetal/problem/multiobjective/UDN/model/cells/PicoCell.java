package org.uma.jmetal.problem.multiobjective.UDN.model.cells;

import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;

/**
 * @author paco
 */
public class PicoCell extends Cell {

    public PicoCell(
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
                workingFrequency);

        this.type_ = UDN.CellType.PICO;
        this.sector_.coverageRadius_ = coverageRadius;
        this.cost_ = 250;
        this.active_ = false;
        //MIMO Capacity Parameters (precomputed)
        this.singularValuesH = new double[]{9.0856, 8.4032, 7.1846, 6.6236};
        this.numAntennasRx = 4;
        this.numAntennasTx = 64;
    }

    public PicoCell(Cell c) {
        this.id_ = c.id_;
        this.bts_ = new BTS(c.bts_);
    }

    @Override
    Cell newInstance() {
        return new PicoCell(this);
    }
}
