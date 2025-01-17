﻿using NUnit.Framework;
using SnowLoads.Tests;
using System;

namespace SnowLoads.BuildingTypes.Tests
{
    [TestFixture()]
    public class MonopitchRoofTests
    {
        [Test()]
        [Description("Check constructor for the monopitchRoof.")]
        public void MonopitchRoofTest_Constructor_Success()
        {
            var monopitchRoof = new MonopitchRoof(new BuildingImplementation()
            { SnowLoad = new SnowLoadImplementation() }, 15);
            Assert.IsNotNull(monopitchRoof, "MonopitchRoof should be created.");
            Assert.AreEqual(15, monopitchRoof.Slope, "Slope should be set at construction time.");
        }

        [Test()]
        [Description("Check calculations of snow loads for the monopitchRoof.")]
        public void MonopitchRoofTest_CalculateSnowLoad_Success()
        {
            var building = BuildingImplementation.CreateBuilding();

            var monopitchRoof = new MonopitchRoof(building, 15);

            monopitchRoof.CalculateSnowLoad();
            Assert.AreEqual(0.72, Math.Round(monopitchRoof.SnowLoadOnRoofValue, 3), "Snow load is not calculated properly.");
        }

        [Test()]
        [Description("Example number 2 from \"Obciążenia budynków i konstrukcji budowlanych według Eurokodów\" - Anna Rawska-Skotniczy")]
        public void ExampleTest2_CalculateSnowLoad_Success()
        {
            var buildingSite = new BuildingSite(ZoneEnum.ThirdZone, TopographyEnum.Normal, 360);
            buildingSite.CalculateExposureCoefficient();
            var snowLoad = new SnowLoad(buildingSite);
            snowLoad.CalculateSnowLoad();
            var building = new Building(snowLoad);
            building.CalculateThermalCoefficient();

            var monopitchRoof = new MonopitchRoof(building, 5);

            monopitchRoof.CalculateSnowLoad();
            Assert.AreEqual(1.248, Math.Round(monopitchRoof.SnowLoadOnRoofValue, 3),
                "Snow load for roof is not calculated properly.");
        }

        [Test()]
        [Description("Example number 4 from \"Obciążenia budynków i konstrukcji budowlanych według Eurokodów\" - Anna Rawska-Skotniczy")]
        public void ExampleTest4_CalculateSnowLoad_Success()
        {
            var buildingSite = new BuildingSite(ZoneEnum.SecondZone, TopographyEnum.Normal, 175);
            buildingSite.CalculateExposureCoefficient();
            var snowLoad = new SnowLoad(buildingSite);
            snowLoad.CalculateSnowLoad();
            var building = new Building(snowLoad);
            building.CalculateThermalCoefficient();

            var monopitchRoof = new MonopitchRoof(building, 10);

            monopitchRoof.CalculateSnowLoad();
            Assert.AreEqual(0.72, Math.Round(monopitchRoof.SnowLoadOnRoofValue, 3),
                "Snow load is not calculated properly.");
        }
    }
}