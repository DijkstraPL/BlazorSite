﻿using SnowLoads.API;
using SnowLoads.BuildingTypes;
using SnowLoads.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SnowLoads.Exceptional
{
    public class ExceptionalMultiSpanRoof : ICalculatable
    {
        #region Properties

        public double ShapeCoefficient { get; private set; }

        public double SnowLoad { get; private set; }

        /// <summary>
        /// Instance of building.
        /// </summary>
        public Building Building { get; private set; }

        public double LeftDriftLength { get; set; }

        public double RightDriftLength { get; set; }

        public double HeightInTheLowerPart { get; private set; }

        public double HorizontalDimensionOfThreeSlopes { get; private set; }

        #endregion // Properties

        #region Fields

        private SnowLoad snowLoad;

        #endregion // Fields

        #region Constructors

        public ExceptionalMultiSpanRoof(Building building, double leftDriftLength, double rightDriftLength)
        {
            Building = building;
            LeftDriftLength = leftDriftLength;
            RightDriftLength = rightDriftLength;
            SetReferences();
        }

        #endregion // Constructors

        #region Methods
        
        public void CalculateSnowLoad()
        {
            SetHorizontalDimensionOfThreeSlopes();
            CalculateShapeCoefficient();
            CalculateSnowLoadOnRoof();
        }
        
        private void SetReferences()
        {
            snowLoad = Building.SnowLoad;
        }

        private void SetHorizontalDimensionOfThreeSlopes()
        {
            HorizontalDimensionOfThreeSlopes = (LeftDriftLength + RightDriftLength) * 1.5;
        }

        private void CalculateShapeCoefficient()
        {
            ShapeCoefficient = Math.Min(
                2 * HeightInTheLowerPart / snowLoad.SnowLoadForSpecificReturnPeriod,
                2 * HorizontalDimensionOfThreeSlopes / (LeftDriftLength + RightDriftLength));

            ShapeCoefficient = Math.Min(ShapeCoefficient, 5);
        }

        private void CalculateSnowLoadOnRoof()
        {
            if (ConditionChecker.ForDesignSituation(snowLoad.ExcepctionalSituation, snowLoad.CurrentDesignSituation, true))
                SnowLoad = SnowLoadCalc.CalculateSnowLoadForAnnexB(ShapeCoefficient, snowLoad.SnowLoadForSpecificReturnPeriod);
        }

        #endregion // Methods
    }
}