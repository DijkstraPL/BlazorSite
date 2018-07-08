﻿using SnowLoads.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SnowLoads.BuildingTypes
{
    /// <summary>
    /// Class for calculation of snow hanging over the roof.
    /// </summary>
    public class SnowOverhanging : ICalculatable
    {
        #region Properties

        /// <summary>
        /// Thickness of the snow.
        /// </summary>
        [Abbreviation("d")]
        public double SnowLayerDepth { get; set; }

        /// <summary>
        /// Shape coefficient.
        /// </summary>
        [Abbreviation("k")]
        public double IrregularShapeCoefficient { get; private set; }

        /// <summary>
        /// Snow load on the roof - the most onerous undrifted load case
        /// appropriate for the roof under consideration [kN/m2].
        /// </summary>
        [Abbreviation("s")]
        public double SnowLoadOnRoofValue { get; set; }

        /// <summary>
        /// Snow load per metre length due to the overhang [kN/m].
        /// </summary>
        [Abbreviation("s_e")]
        public double SnowLoad { get; private set; }

        /// <summary>
        /// Instance of building.
        /// </summary>
        public Building Building { get; private set; }

        #endregion // Properties

        #region Constructor 

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="building">Instance of buildinng.</param>
        public SnowOverhanging(Building building, double snowLayerDepth, double snowLoadOnRoof)
        {
            Building = building;
            SnowLayerDepth = snowLayerDepth;
            SnowLoadOnRoofValue = snowLoadOnRoof;
        }

        #endregion

        #region Methods

        public void CalculateSnowLoad()
        {
            CalculateIrregularShapeCoefficient();
            CalculateOverhangingSnowLoad();
        }

        /// <summary>
        /// Calculate irregular shape coefficient.
        /// </summary>
        private void CalculateIrregularShapeCoefficient()
        {
            IrregularShapeCoefficient = Math.Min(3 / SnowLayerDepth, SnowLayerDepth * Building.SnowLoad.SnowDensity);
        }

        /// <summary>
        /// Calculate snow load.
        /// </summary>
        private void CalculateOverhangingSnowLoad()
        {
            SnowLoad = IrregularShapeCoefficient * Math.Pow(SnowLoadOnRoofValue, 2) / Building.SnowLoad.SnowDensity;
        }

        #endregion // Methods
    }
}