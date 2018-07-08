﻿//using Microsoft.VisualStudio.TestTools.UnitTesting;
using SnowLoads;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NUnit.Framework;

namespace SnowLoads.Tests
{
    [TestFixture()]
    public class BuildingSiteTests
    {
        #region CalculateExposureCoefficient

        [Test()]
        [Description("Ensure that method for calculation of exposure coefficient work correctly.")]
        public void CalculateExposureCoefficientTest_WindsweptTopography_Success()
        {
            var buildingSite = new BuildingSite();
            buildingSite.CurrentTopography = Topography.Windswept;

            buildingSite.CalculateExposureCoefficient();

            Assert.AreEqual(0.8, buildingSite.ExposureCoefficient, "Exposure coefficient is wrong.");
        }

        [Test()]
        [Description("Ensure that method for calculation of exposure coefficient work correctly.")]
        public void CalculateExposureCoefficientTest_NormalTopography_Success()
        {
            var buildingSite = new BuildingSite();
            buildingSite.CurrentTopography = Topography.Normal;

            buildingSite.CalculateExposureCoefficient();

            Assert.AreEqual(1.0, buildingSite.ExposureCoefficient, "Exposure coefficient is wrong.");
        }

        [Test()]
        [Description("Ensure that method for calculation of exposure coefficient work correctly.")]
        public void CalculateExposureCoefficientTest_ShelteredTopography_Success()
        {
            var buildingSite = new BuildingSite();
            buildingSite.CurrentTopography = Topography.Sheltered;

            buildingSite.CalculateExposureCoefficient();

            Assert.AreEqual(1.2, buildingSite.ExposureCoefficient, "Exposure coefficient is wrong.");
        }

        [Test()]
        [Description("Ensure that method for calculation of exposure coefficient work correctly.")]
        public void CalculateExposureCoefficientTest_NoneTopography_Success()
        {
            var buildingSite = new BuildingSite();
            buildingSite.CurrentTopography = Topography.None;

            buildingSite.CalculateExposureCoefficient();

            Assert.AreEqual(1.2, buildingSite.ExposureCoefficient, "Exposure coefficient is wrong.");
        }
        
        #endregion // CalculateExposureCoefficient
    }
}