﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Tools;

namespace SnowLoads.Interfaces
{
    public interface ILengthProvider
    {
        [Abbreviation("l_s")]
        double DriftLength { get; }

        void CaluclateDriftLength();
    }
}
