package org.uma.jmetal.problem.multiobjective.UDN.model.cells;

import org.uma.jmetal.problem.multiobjective.UDN.model.UDN;

/**
 * @author paco
 */
public class MacroCell extends Cell {

    public MacroCell(
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

        this.type_ = UDN.CellType.MACRO;
        this.cost_ = 1000;
        this.active_ = true;
        //MIMO Capacity Parameters (precomputed)
        this.singularValuesH = new double[]{1};
        this.numAntennasRx = 1;
        this.numAntennasTx = 1;
    }

    public MacroCell(Cell c) {
        this.id_ = c.id_;
        this.sector_ = new Sector(c.sector_);
    }

    @Override
    Cell newInstance() {
        return new MacroCell(this);
    }
}
