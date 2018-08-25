﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Tools;

namespace SnowLoads.Interfaces
{
    public interface ITemperatureProvider
    {
        /// <summary>
        /// Internal temperature in degrees of Celsius
        /// </summary>
        /// <remarks>[PN-EN 1991-1-3 NB1.8]</remarks>
        [Abbreviation("t_i")]
        [Unit("C")]
        double InternalTemperature { get; set; }

        /// <summary>
        /// Difference between temperatures
        /// </summary>
        [Abbreviation("delta-t")]
        [Unit("C")]
        double TempreatureDifference { get; }
    }
}