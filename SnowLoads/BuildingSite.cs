﻿using SnowLoads.Interfaces;
using System;
using Tools;

namespace SnowLoads
{
    /// <summary>
    /// Class containing additional informations about building site.
    /// </summary>
    /// <example>
    /// <code>  
    /// class TestClass 
    /// {
    ///     static void Main() 
    ///     {
    ///         BuildingSite buildingSite = new BuildingSite();
    ///         buildingSite.CalculateExposureCoefficient();
    ///     }
    /// }
    /// </code>
    /// </example>
    public class BuildingSite : IBuildingSite
    {
        #region Properties

        /// <summary>
        /// Altitude above sea level.
        /// </summary>
        /// <remarks>
        /// Height above mean sea level of the site where the structure is to be located, or
        /// is already located for an existing structure.
        /// </remarks>
        [Abbreviation("A")]
        [Unit("m")]
        public double AltitudeAboveSea { get; set; }

        /// <summary>
        /// Zone read from the proper map - <see cref="ZoneEnum"/>.
        /// </summary>
        /// <remarks>[PN-EN 1991-1-3 Figure NB.1]</remarks>
        public ZoneEnum CurrentZone { get; set; }

        /// <summary>
        /// Topography base on terrain conditions - <see cref="TopographyEnum"/>.
        /// </summary>
        /// <remarks>[PN-EN 1991-1-3 Table 5.1]</remarks>
        public TopographyEnum CurrentTopography { get; set; }

        /// <summary>
        /// Exposure coefficient.
        /// </summary>
        /// <remarks>
        /// Coefficient defining the reduction or increase of load on a roof of an unheated
        /// building, as a fraction of the characteristic snow load on the ground.
        /// </remarks>
        /// <remarks>[PN-EN 1991-1-3 5.2.(7)]</remarks>
        [Abbreviation("C_e")]
        [Unit("")]
        public double ExposureCoefficient { get; private set; } = 1;

        #endregion // Properties

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="BuildingSite"/> class.
        /// </summary>
        /// <seealso cref="BuildingSite.BuildingSite(ZoneEnum, TopographyEnum, double)"/>
        /// <seealso cref="BuildingSite.BuildingSite(ZoneEnum, TopographyEnum, double, double)"/>
        public BuildingSite() { }

        /// <summary>
        /// Initializes a new instance of the <see cref="BuildingSite"/> class.
        /// </summary>
        /// <param name="currentZone"><see cref="CurrentZone"/> read from proper map.</param>
        /// <param name="currentTopography"><see cref="CurrentTopography"/> based on terrain type.</param>
        /// <param name="altitudeAboveSea"><see cref="AltitudeAboveSea"/> [m].</param>
        /// <seealso cref="BuildingSite.BuildingSite"/>
        /// <seealso cref="BuildingSite.BuildingSite(ZoneEnum, TopographyEnum, double, double)"/>
        public BuildingSite(ZoneEnum currentZone, TopographyEnum currentTopography, double altitudeAboveSea)
        {
            CurrentZone = currentZone;
            CurrentTopography = currentTopography;
            AltitudeAboveSea = altitudeAboveSea;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="BuildingSite"/> class.
        /// </summary>
        /// <param name="currentZone"><see cref="CurrentZone"/> read from proper map.</param>
        /// <param name="currentTopography"><see cref="CurrentTopography"/> based on terrain type.</param>
        /// <param name="altitudeAboveSea"><see cref="AltitudeAboveSea"/> [m].</param>
        /// <param name="exposureCoefficient"><see cref="ExposureCoefficient"/>.</param>
        /// <seealso cref="BuildingSite.BuildingSite"/>
        /// <seealso cref="BuildingSite.BuildingSite(ZoneEnum, TopographyEnum, double)"/>
        public BuildingSite(ZoneEnum currentZone, TopographyEnum currentTopography,
            double altitudeAboveSea, double exposureCoefficient) :
            this(currentZone, currentTopography, altitudeAboveSea)
        {
            ExposureCoefficient = exposureCoefficient;
        }

        #endregion // Constructors

        #region Methods

        /// <summary>
        /// Calcualte <see cref="ExposureCoefficient"/>.
        /// </summary>
        /// <remarks>[PN-EN 1991-1-3 Table 5.1]</remarks>
        /// <exception cref="ArgumentException">Thrown when <see cref="CurrentTopography"/> is not set.</exception>
        public void CalculateExposureCoefficient()
        {
            switch (CurrentTopography)
            {
                case TopographyEnum.Windswept:
                    ExposureCoefficient = 0.8;
                    break;
                case TopographyEnum.Normal:
                    ExposureCoefficient = 1.0;
                    break;
                case TopographyEnum.Sheltered:
                    ExposureCoefficient = 1.2;
                    break;
                case TopographyEnum.None:
                    ExposureCoefficient = 1.2;
                    break;
                default:
                    throw new ArgumentException("Topography should be selected.");
            }
        }

        #endregion // Methods
    }
}
