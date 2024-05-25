using BeamStatica.Loads.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BeamStatica.Loads.PointLoads
{
    public abstract class ConcentratedLoad : ILoad
    {
        public double Value { get; set; }
        public double Position { get; set; }

        public abstract double CalculateShear();
        public abstract double CalculateBendingMoment(double distanceFromLoad);

        protected ConcentratedLoad(double value)
        {
            Value = value;
        }

        protected ConcentratedLoad(double value, double position) : this(value)
        {
            Position = position;
        }
    }
}
